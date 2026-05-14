import asyncio
import json
import logging
import math
import time

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

logger = logging.getLogger("imu")

sensor = None

def get_sensor():
    global sensor
    if sensor is None:
        import board
        from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
        sensor = LSM6DSOX(board.I2C(), address=0x6A)
    return sensor

def reinit_sensor():
    """Re-initialize the sensor after an I2C failure."""
    global sensor
    sensor = None
    return get_sensor()

app = FastAPI()

MAX_I2C_RETRIES = 5
I2C_RETRY_DELAY = 0.05  # seconds


@app.get("/")
async def root():
    return FileResponse("index.html")


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        imu = get_sensor()
    except (ValueError, OSError):
        # No IMU connected — send simulated data so the UI can be tested
        t0 = time.monotonic()
        try:
            while True:
                t = time.monotonic() - t0
                await ws.send_text(json.dumps({
                    "sim": True,
                    "ax": round(0.3 * math.sin(t * 2.0), 4),
                    "ay": round(0.2 * math.cos(t * 1.5), 4),
                    "az": round(9.81 + 0.1 * math.sin(t * 0.8), 4),
                    "gx": round(0.05 * math.sin(t * 3.0), 4),
                    "gy": round(0.04 * math.cos(t * 2.5), 4),
                    "gz": round(0.02 * math.sin(t * 1.2), 4),
                }))
                await asyncio.sleep(0.01)
        except WebSocketDisconnect:
            pass
        return
    consecutive_errors = 0
    try:
        while True:
            try:
                ax, ay, az = imu.acceleration
                gx, gy, gz = imu.gyro
                consecutive_errors = 0
            except OSError as e:
                consecutive_errors += 1
                logger.warning("I2C read error (%d/%d): %s", consecutive_errors, MAX_I2C_RETRIES, e)
                if consecutive_errors >= MAX_I2C_RETRIES:
                    logger.error("Too many consecutive I2C errors, reinitializing sensor")
                    try:
                        imu = reinit_sensor()
                        consecutive_errors = 0
                    except OSError:
                        logger.error("Sensor reinit failed, falling back to simulated data")
                        await ws.send_text(json.dumps({"error": "IMU lost — check wiring"}))
                        return
                await asyncio.sleep(I2C_RETRY_DELAY)
                continue
            await ws.send_text(json.dumps({
                "ax": round(ax, 4), "ay": round(ay, 4), "az": round(az, 4),
                "gx": round(gx, 4), "gy": round(gy, 4), "gz": round(gz, 4),
            }))
            await asyncio.sleep(0.01)
    except WebSocketDisconnect:
        pass
