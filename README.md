# OsirisAvionics

Avionics stack for the Osiris rocket — UC San Diego Rocket Propulsion Lab (RPL).

> **Note:** This is a **practice / sandbox repository**, not the production
> flight codebase. Everything here is experimental: sensor bring-up, driver
> exploration, build infrastructure, and an onboarding ground for new
> avionics team members. Nothing in this tree is flight-ready or
> safety-reviewed. When the avionics matures to a flight build, that code
> will live in a separate repository under the RPL GitHub organization.

## What this is

Firmware, ground-side tooling, and build infrastructure for Osiris avionics
development. The target chip is the **ESP32-P4** (verified from
`firmware/esp-imu/sdkconfig`). Current work is sensor bring-up: reading the
**LSM6DSOX** 6-axis IMU and **BMP388** barometer over I2C, plus a UART
interface to the **Blue Raven** flight computer. Higher-level avionics
functions (telemetry downlink, recovery logic, state estimation) are not
yet in the tree.

Multiple parallel firmware experiments live side-by-side in `firmware/` —
Arduino IDE sketches, current ESP-IDF projects using `driver/i2c_master.h`,
and earlier projects using the deprecated `driver/i2c.h`. This is
intentional for a practice repo: it's a working record of what's been tried.
`firmware/esp-imu/` is the most current ESP-IDF project and the one new
contributors should build on.

## Repository layout

```
docs/                 Loose prose notes (currently nearly empty)
firmware/             On-board firmware
  arduino-ide/        Arduino IDE sketches: sensor tests + Blue Raven UART pass-through
  esp-imu/            Current ESP-IDF project — LSM6DSOX over I2C on ESP32-P4
  imu_esp32/          Older variant of the same, using the deprecated driver/i2c.h API
  sensor_project/     Standalone test.cpp (LSM6DSOX + BMP388 debug stream)
scripts/
  firmware-builders/  Python generators that scaffold ESP-IDF projects + an I2C diag project
  flash-host/         Per-OS setup scripts for a machine that hosts an ESP32 over rfc2217
  osiris-server/      Build-host helper (remote-flash.sh) for flashing over the network
testing/
  hello_world/        Upstream ESP-IDF hello_world example (target: esp32s3) with pytest_embedded
  rpi-imu/            Raspberry Pi 5 FastAPI + WebSocket dashboard reading the same IMU
```

Sub-projects with their own per-directory notes:

- `firmware/esp-imu/CLAUDE.md` — register map, sensor configuration, build/flash quick-reference.
- `testing/rpi-imu/CLAUDE.md` — Pi-side stack details (Adafruit Blinka, FastAPI, frontend).
- `scripts/flash-host/README.md` — what the per-OS setup scripts install and how to run the rfc2217 server.

## Getting started

Assumes a Linux build host. ESP-IDF **v6.0.1** is installed via the
[Espressif Installation Manager (EIM)](https://docs.espressif.com/projects/idf-im-ui/).

```bash
git clone <this-repo-url> OsirisAvionics
cd OsirisAvionics
```

Activate ESP-IDF. The activate script's location depends on where EIM
installed it:

```bash
# Personal machine (default EIM install):
source ~/.espressif/tools/activate_idf_v6.0.1.sh

# osiris-server (shared install at /opt/esp):
source /opt/esp/tools/.espressif/tools/activate_idf_v6.0.1.sh
# Equivalently, on osiris-server, use the `esp` shell alias which
# sources the same file.
```

Build the IMU firmware:

```bash
cd firmware/esp-imu
idf.py set-target esp32p4
idf.py build
```

The flight target is **ESP32-P4**. The `esp32s3` setting in
`testing/hello_world/sdkconfig` is the unchanged upstream default from the
ESP-IDF example and is not load-bearing — that directory exists to verify
the toolchain end-to-end on a fresh host, nothing more.

## Flashing

### Local (board attached over USB)

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

The exact device node varies by board and host:

- `/dev/ttyACM*` — native USB-Serial-JTAG (ESP32-S3, C3, P4)
- `/dev/ttyUSB*` — CP210x / CH340 USB-UART bridge

### Remote (board attached to a different machine)

The build runs on `osiris-server`; the ESP32 is plugged into a teammate's
laptop, which exposes its serial port over the network using esptool's
`esp_rfc2217_server.py`. The build host then flashes and monitors over TCP
without ever touching the device physically.

1. **Device side** — set up the flash host once per machine using the
   per-OS scripts in [`scripts/flash-host/`](scripts/flash-host/) and start
   the rfc2217 server (the script's `Next steps:` output explains how).
2. **Build side** — on osiris-server, after building, run:

   ```bash
   scripts/osiris-server/remote-flash.sh <device-host-ip>
   # or:
   FLASH_HOST=<device-host-ip> scripts/osiris-server/remote-flash.sh
   ```

   This activates ESP-IDF and runs `idf.py flash monitor` against
   `rfc2217://<host>:4000`.

See [`scripts/flash-host/README.md`](scripts/flash-host/README.md) for the
flash-host setup details and production-hardening notes (systemd unit,
stable serial-by-id paths, multi-board ports).

## Testing

The `testing/` directory is presently a sandbox, not an automated test
suite for avionics. Real hardware-in-the-loop tests will land once the
firmware does more than read sensors — for now there's nothing meaningful
to integration-test yet.

- **`testing/hello_world/`** — an unmodified copy of the upstream ESP-IDF
  `hello_world` example, target `esp32s3`. Includes `pytest_hello_world.py`
  (a `pytest_embedded` test that asserts `"Hello world!"` on stdout). Useful
  as a known-good baseline to verify the toolchain and flash chain end-to-end
  on a fresh host.
- **`testing/rpi-imu/`** — a Raspberry Pi 5 web dashboard that reads the
  LSM6DSOX directly via Adafruit Blinka and streams accel+gyro over a
  WebSocket at ~100 Hz. This is a Pi-side experiment for visualizing the
  same sensor used on the flight board, not a test of the ESP firmware.
  Run it with:

  ```bash
  cd testing/rpi-imu
  source .venv/bin/activate
  .venv/bin/python -m uvicorn app:app --host 0.0.0.0 --port 8000
  ```

  Then browse to `http://<pi-ip>:8000`. Falls back to a simulated data
  stream if the IMU isn't detected, so the UI is still usable without
  hardware.

## Contributing

**Avionics co-leads:**

- Edgar Verastegui Medina ([@evm100](https://github.com/evm100))
- Esha Rami ([@esha-rami](https://github.com/esha-rami))

For now: open a PR, or grab a co-lead in person at the lab. New contributors
should ask a co-lead for an invite to the RPL Discord and the
`#osiris-avionics` channel.

When adding firmware:

- Prefer the current ESP-IDF v5.x `driver/i2c_master.h` API
  (see `firmware/esp-imu/main/main.c`); do not use the deprecated
  `driver/i2c.h` API used in `firmware/imu_esp32/` and the Arduino sketches.
- Don't commit `build/`, `managed_components/`, or `dependencies.lock` —
  `.gitignore` already excludes them.
- If you add a new ESP-IDF project, commit its `sdkconfig` so the target
  chip and key options are reproducible.

## License

No `LICENSE` file is currently present at the repo root. For a sandbox
repository used only within the team this is acceptable, but it means
the code has no granted rights for outside contributors. Before this
repo (or any successor flight repository) is migrated to the RPL GitHub
organization or shared publicly, an `MIT` or `Apache-2.0` `LICENSE` file
should be added. GitHub's "Add file → Create new file → LICENSE" flow
provides templates that pre-fill year and copyright holder.
