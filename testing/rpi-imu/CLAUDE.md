# rpi-imu — Raspberry Pi 5 IMU Dashboard

Real-time web dashboard for LSM6DSOX IMU sensor over I2C on Raspberry Pi 5.

## Stack

- **Python 3** with virtual environment at `.venv`
- **Adafruit Blinka + CircuitPython LSM6DS** for sensor access
- **FastAPI + uvicorn** for HTTP/WebSocket server
- **Frontend**: Lightweight real-time charting (SmoothieCharts or uPlot)

## Key Files

- `app.py` — FastAPI backend with WebSocket streaming at ~100Hz
- `index.html` — Real-time dashboard frontend

## Running

```bash
# Activate venv
source .venv/bin/activate

# Start server (accessible from other machines on network)
.venv/bin/python -m uvicorn app:app --host 0.0.0.0 --port 8000

# Find Pi's IP
hostname -I
```

## I2C on Raspberry Pi 5

- Must enable I2C via `sudo raspi-config` (Interface Options > I2C)
- Default I2C bus: bus 1 (pins SDA=GPIO2, SCL=GPIO3)
- Verify with: `i2cdetect -y 1`
- Adafruit Blinka uses `board.I2C()` which auto-detects the bus

## Sensor Notes

- LSM6DSOX sensitivity depends on configured range:
  - Accel 2g: 0.061 mg/LSB, 4g: 0.122 mg/LSB
  - Gyro 250dps: 8.75 mdps/LSB, 2000dps: 70.0 mdps/LSB
- The Adafruit library handles unit conversion — `.acceleration` returns m/s^2, `.gyro` returns rad/s
