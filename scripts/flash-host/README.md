# Flash-Host Setup Scripts

These scripts prepare a machine to serve as a **flash host** — a computer with
an ESP32 plugged in via USB that exposes the serial port over the local network
using esptool's bundled `esp_rfc2217_server.py`. A remote build host (like
osiris-server) can then flash and monitor firmware over the network without
needing the device physically attached.

**Run on the machine with the ESP32 plugged in**, not on the build host.

## Which script

| OS              | Script                          |
|-----------------|---------------------------------|
| Ubuntu / Debian | `setup-flash-host-linux.sh`     |
| macOS           | `setup-flash-host-macos.sh`     |
| Windows 10 / 11 | `setup-flash-host-windows.ps1`  |

## Usage

```bash
# Linux / macOS
chmod +x setup-flash-host-<os>.sh
./setup-flash-host-<os>.sh
```

```powershell
# Windows (run from PowerShell, not cmd.exe)
powershell -ExecutionPolicy Bypass -File .\setup-flash-host-windows.ps1
```

## What gets installed

- **Python 3** (Windows only — usually already present on Linux/macOS)
- **pipx** for isolated Python tool installs
- **esptool**, which includes `esp_rfc2217_server.py`

Total ~50 MB, or ~100 MB on a fresh Windows machine. Scripts are idempotent —
re-running just upgrades existing installs.

## After setup

Each script prints OS-specific instructions for:

1. Finding your ESP32's serial device
2. Starting the rfc2217 server
3. Finding your machine's IP address

Once the server is running, give your IP to whoever's flashing and they run
`remote-flash.sh <your-ip>` from osiris-server.

## Production considerations (later)

For a long-running flash host shared by the team, consider:

- **systemd unit** (Linux) or **launchd plist** (macOS) wrapping
  `esp_rfc2217_server` so the server survives reboots and restarts on
  failure.
- Bind to `/dev/serial/by-id/...` symlinks rather than `/dev/ttyACM0` so
  the server survives USB replug/reorder.
- One server per board on different ports (4000, 4001, ...) if multiple
  ESP32s are attached. Document the port-to-board mapping.
- For off-network access, layer Tailscale on top — the rfc2217 protocol
  works identically over a tailnet IP.
