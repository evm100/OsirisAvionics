# Setup script for the device-side flash host on Windows.
#
# Installs Python (if needed) and esptool, preparing the machine to run
# esp_rfc2217_server.py and serve a locally-attached ESP32 over the
# network for remote flashing from osiris-server.
#
# Run once per machine. Idempotent — safe to re-run.
#
# Usage (from PowerShell, NOT cmd.exe):
#   powershell -ExecutionPolicy Bypass -File .\setup-flash-host-windows.ps1
#
# Or, after enabling script execution permanently for your user:
#   Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
#   .\setup-flash-host-windows.ps1

$ErrorActionPreference = "Stop"

function Write-Step($msg)    { Write-Host $msg -ForegroundColor Cyan }
function Write-Note($msg)    { Write-Host $msg -ForegroundColor Yellow }
function Write-OK($msg)      { Write-Host $msg -ForegroundColor Green }
function Write-Err($msg)     { Write-Host $msg -ForegroundColor Red }

Write-Step "=== RPL remote-flash host setup (Windows) ==="
Write-Host

# --- Python ---
$python = Get-Command python -ErrorAction SilentlyContinue
if (-not $python) {
    Write-Note "[1/3] Python not found."
    $winget = Get-Command winget -ErrorAction SilentlyContinue
    if ($winget) {
        Write-Note "      Installing Python 3.12 via winget..."
        winget install -e --id Python.Python.3.12 --accept-source-agreements --accept-package-agreements
        Write-Note ""
        Write-Note "      Python installed. CLOSE this PowerShell window, open a new one,"
        Write-Note "      and re-run this script. (Required for PATH to refresh.)"
        exit 0
    } else {
        Write-Err "ERROR: winget not available. Install Python manually from:"
        Write-Err "       https://www.python.org/downloads/"
        Write-Err "       Make sure to check 'Add Python to PATH' during install."
        exit 1
    }
} else {
    Write-OK "[1/3] Python found: $($python.Source)"
}

# --- pipx ---
Write-Step "[2/3] Installing/upgrading pipx..."
python -m pip install --user --upgrade pipx
python -m pipx ensurepath | Out-Null

# --- esptool ---
Write-Step "[3/3] Installing esptool..."
$installed = python -m pipx list 2>$null | Select-String -Pattern "esptool" -Quiet
if ($installed) {
    Write-Host "      Already installed — upgrading..."
    python -m pipx upgrade esptool | Out-Null
} else {
    python -m pipx install esptool
}

Write-Host
Write-Step "=== Setup complete ==="
Write-Host
Write-Note "IMPORTANT: open a NEW PowerShell window before continuing,"
Write-Note "so PATH changes take effect."
Write-Host

Write-Host @"
Next steps:

  1. Plug in your ESP32 and find its COM port:
       Get-PnpDevice -Class Ports | Where-Object Status -eq OK

     It will appear as COM3, COM4, etc.

     USB driver notes:
       Native USB-Serial-JTAG (ESP32-S3/C3/P4) — no driver needed
       CP210x bridge — install from Silicon Labs
       CH340/CH341 bridge — install from WCH

  2. Start the rfc2217 server (replace COM3 with your port):
       esp_rfc2217_server -v -p 4000 COM3

  3. Share your machine's IP with whoever's flashing:
       Get-NetIPAddress -AddressFamily IPv4 | Where-Object PrefixOrigin -ne WellKnown

"@
