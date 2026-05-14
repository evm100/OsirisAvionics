#!/bin/bash
#
# Setup script for the device-side flash host on Ubuntu/Debian.
#
# Installs esptool and prepares the system to run esp_rfc2217_server.py,
# which exposes a locally-attached ESP32's serial port over the network
# for remote flashing from osiris-server.
#
# Run once per machine. Idempotent — safe to re-run.
#
# Usage:
#   chmod +x setup-flash-host-linux.sh
#   ./setup-flash-host-linux.sh
#

set -Eeuo pipefail

echo "=== RPL remote-flash host setup (Linux) ==="
echo

# Verify apt-based system
if ! command -v apt &>/dev/null; then
    echo "ERROR: this script requires apt (Debian/Ubuntu/Mint/Pop_OS)."
    echo "For other distros, install python3, python3-pip, and pipx manually,"
    echo "add yourself to the dialout (or equivalent) group, then run:"
    echo "    pipx install esptool"
    exit 1
fi

# Confirm sudo access up front rather than mid-script
if ! sudo -v; then
    echo "ERROR: sudo access required for apt install and dialout group setup."
    exit 1
fi

echo "[1/4] Installing python3, pip, pipx..."
sudo apt-get update -qq
sudo apt-get install -y python3 python3-pip pipx

echo "[2/4] Ensuring ~/.local/bin is on PATH..."
pipx ensurepath >/dev/null

echo "[3/4] Installing esptool via pipx..."
if pipx list 2>/dev/null | grep -qw esptool; then
    echo "      Already installed — upgrading..."
    pipx upgrade esptool >/dev/null
else
    pipx install esptool
fi

echo "[4/4] Adding $USER to dialout group for serial port access..."
NEEDS_RELOGIN=0
if id -nG "$USER" | grep -qw dialout; then
    echo "      Already in dialout group."
else
    sudo usermod -aG dialout "$USER"
    NEEDS_RELOGIN=1
fi

echo
echo "=== Setup complete ==="
echo

if [ "$NEEDS_RELOGIN" = "1" ]; then
    echo "IMPORTANT: log out and back in (or run 'newgrp dialout') for the"
    echo "group change to take effect. Verify with:"
    echo "    groups | grep dialout"
    echo
fi

cat <<'EOF'
Next steps:

  1. Plug in your ESP32 and find its serial device:
       ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

     /dev/ttyACM*  → native USB-Serial-JTAG (ESP32-S3, C3, P4)
     /dev/ttyUSB*  → CP210x/CH340 USB-UART bridge (older boards)

  2. Start the rfc2217 server (replace ttyACM0 with your device):
       esp_rfc2217_server -v -p 4000 /dev/ttyACM0

  3. Share your machine's IP with whoever's flashing.
       ip -4 addr show | grep inet

EOF
