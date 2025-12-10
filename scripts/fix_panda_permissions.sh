#!/bin/bash
# Script to fix USB permissions for panda/jungle devices

set -e

echo "Checking panda/jungle USB device permissions..."

# Check if udev rules exist
if [ ! -f "/etc/udev/rules.d/11-panda.rules" ]; then
    echo "ERROR: /etc/udev/rules.d/11-panda.rules not found!"
    echo "Please run: tools/install_ubuntu_dependencies.sh"
    exit 1
fi

if [ ! -f "/etc/udev/rules.d/12-panda_jungle.rules" ]; then
    echo "ERROR: /etc/udev/rules.d/12-panda_jungle.rules not found!"
    echo "Please run: tools/install_ubuntu_dependencies.sh"
    exit 1
fi

echo "✓ Udev rules found"

# Check for connected devices
DEVICES=$(lsusb | grep -E "(0483|3801|bbaa)" || true)
if [ -z "$DEVICES" ]; then
    echo "No panda/jungle devices detected. Please connect a device and try again."
    exit 1
fi

echo "Found USB devices:"
echo "$DEVICES"
echo ""

# Reload udev rules
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "✓ Udev rules reloaded"
echo ""
echo "If permissions are still not working, try:"
echo "  1. Unplug the USB device"
echo "  2. Wait 2 seconds"
echo "  3. Plug it back in"
echo ""
echo "You can verify permissions with:"
echo "  ls -l /dev/bus/usb/*/* | grep -E '(0483|3801|bbaa)'"


