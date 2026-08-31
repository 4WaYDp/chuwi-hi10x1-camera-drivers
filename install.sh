#!/bin/bash
# Installs the ipu-bridge patch + gc5035 (front) + gc8034 (rear) camera
# drivers as DKMS modules, so they survive kernel upgrades automatically.
#
# Usage: sudo ./install.sh
set -e

if [ "$EUID" -ne 0 ]; then
    echo "Please run this script with sudo: sudo ./install.sh"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for cmd in dkms make; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "Missing required command: $cmd"
        echo "Install with: sudo apt-get install dkms build-essential linux-headers-\$(uname -r)"
        exit 1
    fi
done

if [ ! -d "/lib/modules/$(uname -r)/build" ]; then
    echo "Kernel headers for $(uname -r) not found."
    echo "Install with: sudo apt-get install linux-headers-\$(uname -r)"
    echo "(or the linux-headers-amd64 metapackage, so future kernel upgrades keep working)"
    exit 1
fi

install_dkms_module () {
    local name="$1"
    local srcdir="$SCRIPT_DIR/$name"
    local dest="/usr/src/${name}-1.0"

    echo "=== Installing $name ==="
    dkms remove -m "$name" -v 1.0 --all >/dev/null 2>&1 || true
    rm -rf "$dest"
    cp -r "$srcdir" "$dest"
    dkms add -m "$name" -v 1.0
    dkms build "$name/1.0"
    dkms install "$name/1.0"
}

install_dkms_module ipu-bridge
install_dkms_module gc5035
install_dkms_module gc8034

depmod -a

cat <<'EOF'

Done. All three modules are now DKMS-managed (AUTOINSTALL=yes), so they
will automatically rebuild on future kernel upgrades, as long as matching
kernel headers are installed at upgrade time (linux-headers-amd64
metapackage recommended for that).

Reboot to load the new modules:
  sudo reboot

After rebooting, verify with:
  media-ctl -p          # both gc5035 and gc8034 should show as ENABLED
  cam -l                 # (needs libcamera-tools) should list both cameras

If you'd rather not reboot, you can try a manual reload instead (only if
nothing currently has a camera device open, e.g. no video-call app running):
  sudo modprobe -r gc8034 gc5035 intel_ipu6_isys intel_ipu6 ipu_bridge
  sudo modprobe ipu_bridge intel_ipu6 intel_ipu6_isys gc5035 gc8034
EOF
