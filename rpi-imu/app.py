import asyncio
import json
import math
import time

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

sensor = None

def get_sensor():
    global sensor
    if sensor is None:
        import board
        from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
        sensor = LSM6DSOX(board.I2C())
    return sensor

app = FastAPI()


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
    try:
        while True:
            ax, ay, az = imu.acceleration
            gx, gy, gz = imu.gyro
            await ws.send_text(json.dumps({
                "ax": round(ax, 4), "ay": round(ay, 4), "az": round(az, 4),
                "gx": round(gx, 4), "gy": round(gy, 4), "gz": round(gz, 4),
            }))
            await asyncio.sleep(0.01)
    except WebSocketDisconnect:
        pass
