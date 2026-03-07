# esp-imu — ESP32-P4 LSM6DSOX IMU Firmware

ESP-IDF project that reads LSM6DSOX 6-axis IMU (accelerometer + gyroscope) over I2C on the ESP32-P4.

## Build & Flash

```bash
cd /home/evm100/OsirisAvionics/Firmware/esp-imu
source /home/evm100/esp/esp-idf/export.sh
idf.py set-target esp32p4
idf.py build
idf.py -p /dev/ttyUSBx flash monitor
```

## Hardware

- **MCU**: ESP32-P4
- **Sensor**: LSM6DSOX (I2C address 0x6A)
- **SDA**: GPIO 7
- **SCL**: GPIO 8
- **I2C Frequency**: 400 kHz
- Internal pull-ups enabled (external recommended for production)

## Sensor Configuration

- **Accelerometer**: 104 Hz ODR, +/-4g range (0.122 mg/LSB)
- **Gyroscope**: 104 Hz ODR, 250 dps range (8.75 mdps/LSB)
- Block Data Update (BDU) enabled
- I3C interface disabled
- Software reset on init

## API

Uses the current ESP-IDF v5.x `driver/i2c_master.h` API (NOT the deprecated `driver/i2c.h`).

Key functions:
- `i2c_new_master_bus()` — bus initialization
- `i2c_master_bus_add_device()` — register LSM6DSOX device
- `i2c_master_transmit_receive()` — register reads
- `i2c_master_transmit()` — register writes
- `i2c_master_probe()` — device detection

## Key Registers

| Register | Addr | Purpose |
|----------|------|---------|
| WHO_AM_I | 0x0F | Chip ID (expect 0x6C) |
| CTRL1_XL | 0x10 | Accel ODR + range |
| CTRL2_G  | 0x11 | Gyro ODR + range |
| CTRL3_C  | 0x12 | SW_RESET (bit 0), BDU (bit 6) |
| CTRL9_XL | 0x18 | I3C disable (bit 0) |
| OUTX_L_G | 0x22 | Gyro data start (6 bytes) |
| OUTX_L_A | 0x28 | Accel data start (6 bytes) |

## Related

- `../imu_esp32/` — older version using deprecated I2C API
- `/rpi-imu/` — Raspberry Pi 5 web dashboard (same sensor, Python/Adafruit)
