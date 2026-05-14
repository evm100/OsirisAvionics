#!/bin/bash
#
# Setup script for the device-side flash host on macOS.
#
# Installs esptool via Homebrew + pipx, preparing the machine to run
# esp_rfc2217_server.py and serve a locally-attached ESP32 over the
# network for remote flashing from osiris-server.
#
# Run once per machine. Idempotent — safe to re-run.
#
# Usage:
#   chmod +x setup-flash-host-macos.sh
#   ./setup-flash-host-macos.sh
#

set -Eeuo pipefail

echo "=== RPL remote-flash host setup (macOS) ==="
echo

if ! command -v brew &>/dev/null; then
    echo "ERROR: Homebrew not found. Install it first:"
    echo
    echo '    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"'
    echo
    echo "Then re-run this script."
    exit 1
fi

echo "[1/3] Installing pipx via Homebrew..."
brew install pipx

echo "[2/3] Ensuring pipx bin dir is on PATH..."
pipx ensurepath >/dev/null

echo "[3/3] Installing esptool..."
if pipx list 2>/dev/null | grep -qw esptool; then
    echo "      Already installed — upgrading..."
    pipx upgrade esptool >/dev/null
else
    pipx install esptool
fi

echo
echo "=== Setup complete ==="
echo
echo "If pipx was just added to your PATH, open a new terminal before continuing."
echo

cat <<'EOF'
Next steps:

  1. Plug in your ESP32 and find its serial device:
       ls /dev/cu.usb* 2>/dev/null

     /dev/cu.usbmodem*    → native USB-Serial-JTAG (ESP32-S3, C3, P4)
     /dev/cu.usbserial-*  → CP210x/CH340 USB-UART bridge (older boards)

     NOTE: use cu.* devices, not tty.* — the tty.* variants block until
     a remote DSR signal, which esptool doesn't provide.

  2. Start the rfc2217 server (replace with your actual device path):
       esp_rfc2217_server -v -p 4000 /dev/cu.usbmodem<TAB to autocomplete>

  3. Share your machine's IP with whoever's flashing.
       ipconfig getifaddr en0    # Wi-Fi
       ipconfig getifaddr en1    # Ethernet (varies)

EOF
