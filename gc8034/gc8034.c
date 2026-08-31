// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Chuwi Hi10 X1 GC8034 port.
//
// Ported from the intel/ipu6-drivers gc5035.c driver (same INT3472 power/ACPI
// sequencing framework applies to both sensors on this board), with the
// register init table and chip-ID value extracted from the official Intel
// Windows driver (gc8034.sys, ACPI\GCTI8034, "Camera Rear") via Ghidra
// binary analysis rather than a public datasheet, since no Linux driver or
// datasheet exists for this sensor.
//
// KNOWN ESTIMATE, NOT EXTRACTED: GC8034_MIPI_FREQ. The Windows driver builds
// this at runtime from IPU6 graph-compiler / ACPI SSDB data rather than
// hardcoding it (its graph_settings XML ships pixel_rate_csi="0" as a
// placeholder), so it could not be recovered by static analysis. Computed
// here from required throughput (3280x2464 @ 30fps, 10bpp, 2 lanes) using the
// same blanking-overhead ratio observed in the already-working gc5035
// register tables, and cross-checked against ipu-bridge.c's own entry for
// OVTI8856 (a similar 8MP-class sensor supports up to 720000000 Hz) landing
// very close to the same value. Needs empirical verification on real
// hardware; if capture fails specifically at the CSI2 PHY-sync stage (not an
// I2C error), try adjacent standard values instead.

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Number of lanes supported by this driver (same as the front GC5035, not
 * independently confirmed for GC8034 - verify against dmesg CSI2 errors). */
#define GC8034_DATA_LANES				2
/* Bits per sample of sensor output */
#define GC8034_BITS_PER_SAMPLE				10

/* See "KNOWN ESTIMATE, NOT EXTRACTED" note above. */
#define GC8034_MIPI_FREQ	720000000LL

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define GC8034_PIXEL_RATE	(GC8034_MIPI_FREQ * 2LL * GC8034_DATA_LANES / GC8034_BITS_PER_SAMPLE)

#define GC8034_NATIVE_WIDTH				3280
#define GC8034_NATIVE_HEIGHT				2464

/* Chip ID - verified by decompiling gc8034.sys's sensor-id-check routine:
 * it reads regs 0xf0 (H) / 0xf1 (L) and compares against 0x8044 - NOT
 * 0x8034 as the marketing part number would suggest. */
#define GC8034_REG_CHIP_ID_H				0xf0
#define GC8034_REG_CHIP_ID_L				0xf1
#define GC8034_CHIP_ID					0x8044
#define GC8034_ID(_msb, _lsb)				((_msb) << 8 | (_lsb))

/* Register page selection register (same convention as GC5035) */
#define GC8034_PAGE_REG					0xfe

/*
 * Control register addresses below are NOT independently reverse-engineered
 * bit-for-bit - they're reused from GC5035 because the extracted GC8034
 * global init table writes the exact same addresses (0xb0/0xb1/0xb2/0xb6 for
 * gain, 0x03/0x04 for exposure, 0x41/0x42 for VTS) in the same page-0
 * layout, which is strong structural evidence both sensors share GalaxyCore's
 * generic control register map for this generation. The streaming mode
 * register was originally assumed by the same convention too (0x3e/0x91,
 * copied from GC5035) but that has since been proven wrong - see the verified
 * block below GC8034_PAGE_REG's sibling defines further down.
 */
#define GC8034_REG_EXPOSURE_H				0x03
#define GC8034_REG_EXPOSURE_L				0x04
#define GC8034_EXPOSURE_MIN				4
#define GC8034_EXPOSURE_STEP				1

#define GC8034_REG_ANALOG_GAIN				0xb6
#define GC8034_ANALOG_GAIN_MIN				256
#define GC8034_ANALOG_GAIN_MAX				(16 * GC8034_ANALOG_GAIN_MIN)
#define GC8034_ANALOG_GAIN_STEP				1
#define GC8034_ANALOG_GAIN_DEFAULT			GC8034_ANALOG_GAIN_MIN

#define GC8034_REG_DIGI_GAIN_H				0xb1
#define GC8034_REG_DIGI_GAIN_L				0xb2
#define GC8034_DIGI_GAIN_MIN				0
#define GC8034_DIGI_GAIN_MAX				1023
#define GC8034_DIGI_GAIN_STEP				1
#define GC8034_DIGI_GAIN_DEFAULT			GC8034_DIGI_GAIN_MAX

#define GC8034_REG_VTS_H				0x41
#define GC8034_REG_VTS_L				0x42
#define GC8034_VTS_MAX					16383
#define GC8034_EXPOSURE_MARGIN				16

/*
 * VERIFIED 2026-08-30 by re-decompiling gc8034.sys (FUN_140009438, a plain
 * if (enable) {write table B} else {write table A} function - i.e. the real
 * SetStream(bool) routine) and reading its two register-write tables directly
 * out of the binary's .rdata section: {0xfe,0x00},{0x3f,0x00} for disable and
 * {0xfe,0x00},{0x3f,0xd0} for enable. This is NOT the GC5035-inherited
 * 0x3e/0x91 convention originally assumed here - that guess was wrong and is
 * the likely root cause of the silent capture hang (the sensor never actually
 * received a stream-start command, since it was never written to 0x3e at
 * all). Confirmed there is no 0x3e write anywhere in gc8034.sys's extracted
 * register-write tables.
 */
#define GC8034_REG_CTRL_MODE				0x3f
#define GC8034_MODE_SW_STANDBY				0x00
#define GC8034_MODE_STREAMING				0xd0

/* The clock source index in INT3472 CLDB (identical mechanism to GC5035) */
#define INT3472_CLDB_CLKSRC_INDEX 	14

/*
 * 82c0d13a-78c5-4244-9bb1-eb8b539a8d11
 * This _DSM GUID calls CLKC and CLKF.
 */
static const guid_t clock_ctrl_guid =
	GUID_INIT(0x82c0d13a, 0x78c5, 0x4244,
		  0x9b, 0xb1, 0xeb, 0x8b, 0x53, 0x9a, 0x8d, 0x11);

static const char * const gc8034_supplies[] = {
	/*
	 * Same board family as GC5035: a single GPIO-gated "avdd" rail via
	 * INT3472 gates power for I/O, digital core and analog domains
	 * together. Request the same supply under all the names this driver
	 * uses so every regulator_get resolves to that single real regulator.
	 */
	"avdd",
	"avdd",
};

struct gc8034_regval {
	u8 addr;
	u8 val;
};

struct gc8034_power_ctrl {
	struct acpi_device *ctrl_logic;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwren_gpio;
	struct gpio_desc *pled_gpio;

	int status;
	u8 clk_source_index;
};

struct gc8034 {
	struct i2c_client *client;
	struct clk *mclk;
	unsigned long mclk_rate;
	struct regulator *iovdd_supply;
	struct regulator_bulk_data supplies[ARRAY_SIZE(gc8034_supplies)];

	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;

	/*
	 * Serialize control access, get/set format, get selection
	 * and start streaming.
	 */
	struct mutex mutex;
	struct gc8034_power_ctrl power;
	bool streaming;
};

static inline struct gc8034 *to_gc8034(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc8034, subdev);
}

/*
 * Global init sequence extracted verbatim from the official Windows driver
 * (gc8034.sys) via Ghidra: found at file offset 0x1e260 as 189 consecutive
 * {u32 length, u32 addr, u32 value, u32 reserved=0} 16-byte records - the
 * same record layout confirmed (byte-for-byte, against the known-correct
 * open-source gc5035.c table) in gc5035.sys at offset 0x1ec90. Register
 * order (f2,f4,f5,f6,f8,fa,f9,f7,fc,fc,fc,fe...) follows the same GalaxyCore
 * bank/page-select convention as GC5035's own init table.
 *
 * Only one contiguous table was found (unlike GC5035's two ~160-entry
 * tables for different sub-modes), consistent with GC8034's
 * graph_settings_GC8034_GC8034_ADL.xml declaring exactly one sensor_mode
 * ("full", 3280x2464) - i.e. no separate binned/cropped mode tables exist
 * to port for this sensor/tuning revision.
 */
static const struct gc8034_regval gc8034_global_regs[] = {
	{0xf2, 0x00},
	{0xf4, 0x80},
	{0xf5, 0x19},
	{0xf6, 0x33},
	{0xf8, 0x63},
	{0xfa, 0x45},
	{0xf9, 0x00},
	{0xf7, 0x95},
	{0xfc, 0x00},
	{0xfc, 0x00},
	{0xfc, 0xea},
	{0xfe, 0x03},
	{0x03, 0x9a},
	{0x18, 0x07},
	{0x01, 0x07},
	{0xfc, 0xee},
	{0xfe, 0x00},
	{0x88, 0x03},
	{0xfe, 0x00},
	{0x03, 0x08},
	{0x04, 0xc6},
	{0x05, 0x02},
	{0x06, 0x16},
	{0x07, 0x00},
	{0x08, 0x10},
	{0x09, 0x00},
	{0x0a, 0x3a},
	{0x0b, 0x00},
	{0x0c, 0x04},
	{0x0d, 0x09},
	{0x0e, 0xa0},
	{0x0f, 0x0c},
	{0x10, 0xd4},
	{0x17, 0xc0},
	{0x18, 0x02},
	{0x19, 0x17},
	{0x1e, 0x50},
	{0x1f, 0x80},
	{0x21, 0x4c},
	{0x25, 0x00},
	{0x28, 0x4a},
	{0x2d, 0x89},
	{0xca, 0x02},
	{0xcb, 0x00},
	{0xcc, 0x39},
	{0xce, 0xd0},
	{0xcf, 0x93},
	{0xd0, 0x1b},
	{0xd1, 0xaa},
	{0xd2, 0xcb},
	{0xd8, 0x40},
	{0xd9, 0xff},
	{0xda, 0x0e},
	{0xdb, 0xb0},
	{0xdc, 0x0e},
	{0xde, 0x08},
	{0xe4, 0xc6},
	{0xe5, 0x08},
	{0xe6, 0x10},
	{0xed, 0x2a},
	{0xfe, 0x02},
	{0x59, 0x02},
	{0x5a, 0x04},
	{0x5b, 0x08},
	{0x5c, 0x20},
	{0xfe, 0x00},
	{0x1a, 0x09},
	{0x1d, 0x13},
	{0xfe, 0x10},
	{0xfe, 0x00},
	{0xfe, 0x10},
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0x20, 0x55},
	{0x33, 0x83},
	{0xfe, 0x01},
	{0xdf, 0x06},
	{0xe7, 0x18},
	{0xe8, 0x20},
	{0xe9, 0x16},
	{0xea, 0x17},
	{0xeb, 0x50},
	{0xec, 0x6c},
	{0xed, 0x9b},
	{0xee, 0xd8},
	{0xfe, 0x00},
	{0x80, 0x13},
	{0x84, 0x01},
	{0x89, 0x03},
	{0x8d, 0x03},
	{0x8f, 0x14},
	{0xad, 0x00},
	{0x66, 0x0c},
	{0xbc, 0x09},
	{0xc2, 0x7f},
	{0xc3, 0xff},
	{0x90, 0x01},
	{0x92, 0x00},
	{0x94, 0x00},
	{0x95, 0x09},
	{0x96, 0xa0},
	{0x97, 0x0c},
	{0x98, 0xd0},
	{0xb0, 0x90},
	{0xb1, 0x01},
	{0xb2, 0x00},
	{0xb6, 0x00},
	{0xfe, 0x00},
	{0x40, 0x22},
	{0x41, 0x20},
	{0x42, 0x02},
	{0x43, 0x08},
	{0x4e, 0x0f},
	{0x4f, 0xf0},
	{0x58, 0x80},
	{0x59, 0x80},
	{0x5a, 0x80},
	{0x5b, 0x80},
	{0x5c, 0x00},
	{0x5d, 0x00},
	{0x5e, 0x00},
	{0x5f, 0x00},
	{0x6b, 0x01},
	{0x6c, 0x00},
	{0x6d, 0x0c},
	{0xfe, 0x01},
	{0xbf, 0x40},
	{0xfe, 0x01},
	{0x68, 0x77},
	{0xfe, 0x01},
	{0x60, 0x00},
	{0x61, 0x10},
	{0x62, 0x60},
	{0x63, 0x30},
	{0x64, 0x00},
	{0xfe, 0x01},
	{0xa0, 0x12},
	{0xa8, 0x60},
	{0xa2, 0xd1},
	{0xc8, 0x57},
	{0xa1, 0xb8},
	{0xa3, 0x91},
	{0xc0, 0x50},
	{0xd0, 0x05},
	{0xd1, 0xb2},
	{0xd2, 0x1f},
	{0xd3, 0x00},
	{0xd4, 0x00},
	{0xd5, 0x00},
	{0xd6, 0x00},
	{0xd7, 0x00},
	{0xd8, 0x00},
	{0xd9, 0x00},
	{0xa4, 0x10},
	{0xa5, 0x20},
	{0xa6, 0x60},
	{0xa7, 0x80},
	{0xab, 0x18},
	{0xc7, 0xc0},
	{0xfe, 0x01},
	{0xda, 0x00},
	{0xdb, 0x00},
	{0xdc, 0x00},
	{0xdd, 0x00},
	{0xfe, 0x01},
	{0x20, 0x02},
	{0x21, 0x02},
	{0x23, 0x42},
	{0xfe, 0x03},
	{0x02, 0x03},
	{0x04, 0x80},
	{0x11, 0x2b},
	{0x12, 0x04},
	{0x13, 0x10},
	{0x15, 0x12},
	{0x16, 0x29},
	{0x17, 0xff},
	{0x19, 0xaa},
	{0x1a, 0x02},
	{0x21, 0x05},
	{0x22, 0x06},
	{0x23, 0x2b},
	{0x24, 0x00},
	{0x25, 0x12},
	{0x26, 0x07},
	{0x29, 0x07},
	{0x2a, 0x12},
	{0x2b, 0x07},
	{0xfe, 0x00},
};

static const char * const gc8034_test_pattern_menu[] = {
	"Disabled",
	"Color Bar",
};

static const s64 gc8034_link_freqs[] = {
	GC8034_MIPI_FREQ,
};

static struct gpio_desc *gc8034_get_gpio(struct gc8034 *gc8034,
					  const char *name)
{
	struct device *dev = &gc8034->client->dev;
	struct gpio_desc *gpio;
	int ret;

	gpio = devm_gpiod_get(dev, name, GPIOD_OUT_HIGH);
	ret = PTR_ERR_OR_ZERO(gpio);
	if (ret < 0) {
		gpio = NULL;
		dev_warn(dev, "failed to get %s gpio: %d\n", name, ret);
	}

	return gpio;
}

static void gc8034_init_power_ctrl(struct gc8034 *gc8034)
{
	struct gc8034_power_ctrl *power = &gc8034->power;
	acpi_handle handle = ACPI_HANDLE(&gc8034->client->dev);
	struct acpi_handle_list dep_devices;
	acpi_status status;
	int i;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object *obj;

	power->ctrl_logic = NULL;
	if (!acpi_has_method(handle, "_DEP"))
		return;

	if (!acpi_evaluate_reference(handle, "_DEP", NULL, &dep_devices))
		return;

	for (i = 0; i < dep_devices.count; i++) {
		struct acpi_device *dep_device = NULL;
		const char *dep_hid = NULL;

		if (dep_devices.handles[i]) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0)
			acpi_bus_get_device(dep_devices.handles[i], &dep_device);
#else
			dep_device = acpi_fetch_acpi_dev(dep_devices.handles[i]);
#endif
		}
		if (dep_device)
			dep_hid = acpi_device_hid(dep_device);
		if (dep_hid && strcmp("INT3472", dep_hid) == 0) {
			power->ctrl_logic = dep_device;
			break;
		}
	}

	if (power->ctrl_logic == NULL)
		return;

	status = acpi_evaluate_object(power->ctrl_logic->handle,
				      "CLDB", NULL, &buffer);
	if (ACPI_FAILURE(status)) {
		dev_warn(&gc8034->client->dev, "Read INT3472 CLDB failed");
		return;
	}

	obj = buffer.pointer;
	if (!obj)
		dev_warn(&gc8034->client->dev, "INT3472 CLDB return NULL");
	if (obj->type != ACPI_TYPE_BUFFER) {
		acpi_handle_err(power->ctrl_logic->handle,
				"CLDB object is not an ACPI buffer\n");
		kfree(obj);
		return;
	}
	if (obj->buffer.length < INT3472_CLDB_CLKSRC_INDEX + 1) {
		acpi_handle_err(power->ctrl_logic->handle,
				"The CLDB buffer size is wrong\n");
		kfree(obj);
		return;
	}

	gc8034->power.clk_source_index =
		obj->buffer.pointer[INT3472_CLDB_CLKSRC_INDEX];
	kfree(obj);

	power->reset_gpio = gc8034_get_gpio(gc8034, "reset");
	power->pwren_gpio = gc8034_get_gpio(gc8034, "pwren");
	power->pled_gpio = gc8034_get_gpio(gc8034, "pled");
	power->status = 0;
}

static void gc8034_set_power(struct gc8034 *gc8034, int on)
{
	struct gc8034_power_ctrl *power = &gc8034->power;

	on = (on ? 1 : 0);
	if (on == power->status)
		return;

	if (power->reset_gpio) {
		gpiod_set_value_cansleep(power->reset_gpio, 0);
		msleep(5);
	}

	if (power->ctrl_logic) {
		u8 clock_args[] = { power->clk_source_index, on, 0x01,};
		union acpi_object clock_ctrl_args = {
			.buffer = {
				.type = ACPI_TYPE_BUFFER,
				.length = 3,
				.pointer = clock_args,
			},
		};
		acpi_evaluate_dsm(power->ctrl_logic->handle,
				  &clock_ctrl_guid, 0x00, 0x01,
				  &clock_ctrl_args);
	}

	if (power->pwren_gpio) {
		gpiod_set_value_cansleep(power->pwren_gpio, on);
	} else if (!IS_ERR_OR_NULL(gc8034->iovdd_supply)) {
		int ret;

		if (on)
			ret = regulator_enable(gc8034->iovdd_supply);
		else
			ret = regulator_disable(gc8034->iovdd_supply);
		if (ret)
			dev_warn(&gc8034->client->dev,
				 "Failed to %s avdd regulator: %d\n",
				 on ? "enable" : "disable", ret);
	}
	if (power->pled_gpio)
		gpiod_set_value_cansleep(power->pled_gpio, on);

	if (on && power->reset_gpio) {
		gpiod_set_value_cansleep(power->reset_gpio, 1);
		msleep(5);
	}
	power->status = on;
}

static int gc8034_write_reg(struct gc8034 *gc8034, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(gc8034->client, reg, val);
}

static int gc8034_write_array(struct gc8034 *gc8034,
			      const struct gc8034_regval *regs,
			      size_t num_regs)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_regs; i++) {
		ret = gc8034_write_reg(gc8034, regs[i].addr, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int gc8034_read_reg(struct gc8034 *gc8034, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(gc8034->client, reg);
	if (ret < 0)
		return ret;

	*val = (unsigned char)ret;

	return 0;
}

static int gc8034_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	s64 h_blank, vblank_def;
	/* Single supported mode - see graph_settings XML note above. */
	const u32 width = 3280, height = 2464;
	const u32 hts_def = 3600, vts_def = 2560;

	fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->format.width = width;
	fmt->format.height = height;
	fmt->format.field = V4L2_FIELD_NONE;

	mutex_lock(&gc8034->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
#else
		*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;
#endif
	} else {
		h_blank = hts_def - width;
		__v4l2_ctrl_modify_range(gc8034->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = round_up(vts_def, 4) - height;
		__v4l2_ctrl_modify_range(gc8034->vblank, vblank_def,
					 GC8034_VTS_MAX - height,
					 1, vblank_def);
	}
	mutex_unlock(&gc8034->mutex);

	return 0;
}

static int gc8034_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct gc8034 *gc8034 = to_gc8034(sd);

	mutex_lock(&gc8034->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
		fmt->format = *v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
#else
		fmt->format = *v4l2_subdev_state_get_format(sd_state, fmt->pad);
#endif
	} else {
		fmt->format.width = 3280;
		fmt->format.height = 2464;
		fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&gc8034->mutex);

	return 0;
}

static int gc8034_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = GC8034_NATIVE_WIDTH;
		sel->r.height = GC8034_NATIVE_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int gc8034_enum_mbus_code(struct v4l2_subdev *sd,
			         struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;

	return 0;
}

static int gc8034_enum_frame_sizes(struct v4l2_subdev *sd,
			           struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index != 0)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width  = 3280;
	fse->max_width  = 3280;
	fse->max_height = 2464;
	fse->min_height = 2464;

	return 0;
}

static int __gc8034_start_stream(struct gc8034 *gc8034)
{
	int ret;

	gc8034_set_power(gc8034, 1);

	ret = gc8034_write_array(gc8034, gc8034_global_regs,
				 ARRAY_SIZE(gc8034_global_regs));
	if (ret)
		return ret;

	ret = __v4l2_ctrl_handler_setup(&gc8034->ctrl_handler);
	if (ret)
		return ret;

	ret = gc8034_write_reg(gc8034, GC8034_PAGE_REG, 0);
	if (ret)
		return ret;

	return gc8034_write_reg(gc8034, GC8034_REG_CTRL_MODE,
				GC8034_MODE_STREAMING);
}

static void __gc8034_stop_stream(struct gc8034 *gc8034)
{
	int ret;
	struct i2c_client *client = gc8034->client;

	ret = gc8034_write_reg(gc8034, GC8034_PAGE_REG, 0);
	if (ret)
		dev_err(&client->dev, "failed to stop streaming!");

	if (gc8034_write_reg(gc8034, GC8034_REG_CTRL_MODE,
				GC8034_MODE_SW_STANDBY))
		dev_err(&client->dev, "failed to stop streaming");
	gc8034_set_power(gc8034, 0);
}

static int gc8034_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	struct i2c_client *client = gc8034->client;
	int ret = 0;

	mutex_lock(&gc8034->mutex);
	on = !!on;
	if (on == gc8034->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __gc8034_start_stream(gc8034);
		if (ret) {
			dev_err(&client->dev, "start stream failed\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__gc8034_stop_stream(gc8034);
		pm_runtime_put(&client->dev);
	}

	gc8034->streaming = on;

unlock_and_return:
	mutex_unlock(&gc8034->mutex);

	return ret;
}

static int gc8034_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc8034 *gc8034 = to_gc8034(sd);
	int ret;

	if (gc8034->streaming) {
		ret = __gc8034_start_stream(gc8034);
		if (ret)
			goto error;
	}

	return 0;

error:
	__gc8034_stop_stream(gc8034);
	gc8034->streaming = false;

	return ret;
}

static int gc8034_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc8034 *gc8034 = to_gc8034(sd);

	if (gc8034->streaming)
		__gc8034_stop_stream(gc8034);

	return 0;
}

static int gc8034_entity_init_cfg(struct v4l2_subdev *subdev,
			        struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.format = {
			.width = 3280,
			.height = 2464,
		}
	};

	gc8034_set_fmt(subdev, sd_state, &fmt);

	return 0;
}

static const struct dev_pm_ops gc8034_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(gc8034_runtime_suspend,
			   gc8034_runtime_resume, NULL)
};

static const struct v4l2_subdev_video_ops gc8034_video_ops = {
	.s_stream = gc8034_s_stream,
};

static const struct v4l2_subdev_pad_ops gc8034_pad_ops = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
	.init_cfg = gc8034_entity_init_cfg,
#endif
	.enum_mbus_code = gc8034_enum_mbus_code,
	.enum_frame_size = gc8034_enum_frame_sizes,
	.get_fmt = gc8034_get_fmt,
	.set_fmt = gc8034_set_fmt,
	.get_selection = gc8034_get_selection,
};

static const struct v4l2_subdev_ops gc8034_subdev_ops = {
	.video	= &gc8034_video_ops,
	.pad	= &gc8034_pad_ops,
};

static const struct media_entity_operations gc8034_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
static const struct v4l2_subdev_internal_ops gc8034_internal_ops = {
	.init_state = gc8034_entity_init_cfg,
};
#endif

static int gc8034_set_exposure(struct gc8034 *gc8034, u32 val)
{
	int ret;

	ret = gc8034_write_reg(gc8034, GC8034_PAGE_REG, 0);
	if (ret)
		return ret;

	ret = gc8034_write_reg(gc8034, GC8034_REG_EXPOSURE_H,
			       (val >> 8) & 0x3f);
	if (ret)
		return ret;

	return gc8034_write_reg(gc8034, GC8034_REG_EXPOSURE_L, val & 0xff);
}

static int gc8034_set_analogue_gain(struct gc8034 *gc8034, u32 a_gain)
{
	/*
	 * Analog-gain-to-register mapping is sensor-tuning-specific and was
	 * not reverse engineered (the AGC lookup table lives in the .aiqb
	 * tuning blob, not the .sys driver). Placeholder: write the raw
	 * value directly and clamp to range; revisit with a real AGC curve
	 * once basic streaming is confirmed working.
	 */
	int ret;

	if (a_gain < GC8034_ANALOG_GAIN_MIN)
		a_gain = GC8034_ANALOG_GAIN_MIN;
	else if (a_gain > GC8034_ANALOG_GAIN_MAX)
		a_gain = GC8034_ANALOG_GAIN_MAX;

	ret = gc8034_write_reg(gc8034, GC8034_PAGE_REG, 0);
	if (ret)
		return ret;

	return gc8034_write_reg(gc8034, GC8034_REG_ANALOG_GAIN,
				a_gain * 16 / GC8034_ANALOG_GAIN_MAX);
}

static int gc8034_set_vblank(struct gc8034 *gc8034, u32 val)
{
	int frame_length = val + 2464;
	int ret;

	ret = gc8034_write_reg(gc8034, GC8034_PAGE_REG, 0);
	if (ret)
		return ret;

	ret = gc8034_write_reg(gc8034, GC8034_REG_VTS_H,
			       (frame_length >> 8) & 0x3f);
	if (ret)
		return ret;

	return gc8034_write_reg(gc8034, GC8034_REG_VTS_L, frame_length & 0xff);
}

static int gc8034_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc8034 *gc8034 = container_of(ctrl->handler,
					     struct gc8034, ctrl_handler);
	struct i2c_client *client = gc8034->client;
	s64 max;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		max = 2464 + ctrl->val - GC8034_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(gc8034->exposure,
					 gc8034->exposure->minimum, max,
					 gc8034->exposure->step,
					 gc8034->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = gc8034_set_exposure(gc8034, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = gc8034_set_analogue_gain(gc8034, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_VBLANK:
		ret = gc8034_set_vblank(gc8034, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	};

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc8034_ctrl_ops = {
	.s_ctrl = gc8034_set_ctrl,
};

static int gc8034_initialize_controls(struct gc8034 *gc8034)
{
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	u32 h_blank, vblank_def;
	u64 exposure_max;
	const u32 width = 3280, height = 2464;
	const u32 hts_def = 3600, vts_def = 2560, exp_def = 0x400;
	int ret;

	handler = &gc8034->ctrl_handler;
	ret = v4l2_ctrl_handler_init(handler, 7);
	if (ret)
		return ret;

	handler->lock = &gc8034->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, gc8034_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
		0, GC8034_PIXEL_RATE, 1, GC8034_PIXEL_RATE);

	h_blank = hts_def - width;
	gc8034->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (gc8034->hblank)
		gc8034->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = round_up(vts_def, 4) - height;
	gc8034->vblank = v4l2_ctrl_new_std(handler, &gc8034_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   GC8034_VTS_MAX - height,
					   4, vblank_def);

	exposure_max = vts_def - GC8034_EXPOSURE_MARGIN;
	gc8034->exposure = v4l2_ctrl_new_std(handler, &gc8034_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     GC8034_EXPOSURE_MIN, exposure_max,
					     GC8034_EXPOSURE_STEP,
					     exp_def);

	v4l2_ctrl_new_std(handler, &gc8034_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  GC8034_ANALOG_GAIN_MIN, GC8034_ANALOG_GAIN_MAX,
			  GC8034_ANALOG_GAIN_STEP, GC8034_ANALOG_GAIN_DEFAULT);

	v4l2_ctrl_new_std(handler, &gc8034_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  GC8034_DIGI_GAIN_MIN, GC8034_DIGI_GAIN_MAX,
			  GC8034_DIGI_GAIN_STEP, GC8034_DIGI_GAIN_DEFAULT);

	if (handler->error) {
		ret = handler->error;
		dev_err(&gc8034->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc8034->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int gc8034_check_sensor_id(struct gc8034 *gc8034,
				  struct i2c_client *client)
{
	struct device *dev = &gc8034->client->dev;
	u16 id;
	u8 pid = 0;
	u8 ver = 0;
	int ret;

	ret = gc8034_read_reg(gc8034, GC8034_REG_CHIP_ID_H, &pid);
	if (ret)
		return ret;

	ret = gc8034_read_reg(gc8034, GC8034_REG_CHIP_ID_L, &ver);
	if (ret)
		return ret;

	id = GC8034_ID(pid, ver);
	if (id != GC8034_CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x)\n", id);
		return -EINVAL;
	}

	dev_dbg(dev, "Detected GC%04x sensor\n", id);

	return 0;
}

static int gc8034_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc8034 *gc8034;
	struct v4l2_subdev *sd;
	int ret, i;
	u32 freq = 192000000UL;

	gc8034 = devm_kzalloc(dev, sizeof(*gc8034), GFP_KERNEL);
	if (!gc8034)
		return -ENOMEM;

	gc8034->client = client;

	gc8034->iovdd_supply = devm_regulator_get(dev, "avdd");
	if (IS_ERR(gc8034->iovdd_supply))
		return dev_err_probe(dev, PTR_ERR(gc8034->iovdd_supply),
				     "Failed to get iovdd regulator\n");

	for (i = 0; i < ARRAY_SIZE(gc8034_supplies); i++)
		gc8034->supplies[i].supply = gc8034_supplies[i];
	ret = devm_regulator_bulk_get(&gc8034->client->dev,
				       ARRAY_SIZE(gc8034_supplies),
				       gc8034->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	gc8034_init_power_ctrl(gc8034);
	gc8034_set_power(gc8034, 1);

	/* mclk is never actually assigned (matches gc5035's own dead-code
	 * path) - real clock enable happens via the direct ACPI _DSM call
	 * in gc8034_set_power() above. */
	ret = clk_set_rate(gc8034->mclk, freq);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to set mclk rate\n");
	gc8034->mclk_rate = clk_get_rate(gc8034->mclk);
	if (gc8034->mclk_rate != freq)
		dev_warn(dev, "mclk rate set to %lu instead of requested %u\n",
			 gc8034->mclk_rate, freq);

	mutex_init(&gc8034->mutex);
	sd = &gc8034->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc8034_subdev_ops);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	sd->internal_ops = &gc8034_internal_ops;
#endif
	ret = gc8034_initialize_controls(gc8034);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to initialize controls\n");
		goto err_destroy_mutex;
	}
	ret = gc8034_runtime_resume(dev);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to power on\n");
		goto err_free_handler;
	}
	ret = gc8034_check_sensor_id(gc8034, client);
	if (ret) {
		dev_err_probe(dev, ret, "Sensor ID check failed\n");
		goto err_power_off;
	}

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &gc8034_subdev_entity_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	gc8034->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &gc8034->pad);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to initialize pads\n");
		goto err_power_off;
	}
	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret) {
		dev_err_probe(dev, ret, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
	gc8034_set_power(gc8034, 0);

	return 0;

err_clean_entity:
	media_entity_cleanup(&sd->entity);
err_power_off:
	gc8034_runtime_suspend(dev);
err_free_handler:
	v4l2_ctrl_handler_free(&gc8034->ctrl_handler);
	gc8034_set_power(gc8034, 0);
err_destroy_mutex:
	mutex_destroy(&gc8034->mutex);

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int gc8034_remove(struct i2c_client *client)
#else
static void gc8034_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc8034 *gc8034 = to_gc8034(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc8034->ctrl_handler);
	mutex_destroy(&gc8034->mutex);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		gc8034_runtime_suspend(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
	#endif
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id gc8034_acpi_ids[] = {
	{"GCTI8034"},
	{}
};

MODULE_DEVICE_TABLE(acpi, gc8034_acpi_ids);
#endif

static const struct of_device_id gc8034_of_match[] = {
	{ .compatible = "galaxycore,gc8034" },
	{},
};
MODULE_DEVICE_TABLE(of, gc8034_of_match);

static struct i2c_driver gc8034_i2c_driver = {
	.driver = {
		.name = "gc8034",
		.pm = &gc8034_pm_ops,
		.acpi_match_table = ACPI_PTR(gc8034_acpi_ids),
		.of_match_table = gc8034_of_match,
	},
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
	.probe_new	= gc8034_probe,
#else
	.probe		= gc8034_probe,
#endif
	.remove		= gc8034_remove,
};
module_i2c_driver(gc8034_i2c_driver);

MODULE_DESCRIPTION("GalaxyCore gc8034 sensor driver");
MODULE_LICENSE("GPL v2");
