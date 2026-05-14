#!/bin/bash

#To start the server:

source .venv/bin/activate
.venv/bin/python -m uvicorn app:app --host 0.0.0.0 --port 8000

#  To find your Pi's IP (for viewing from another machine):

# hostname -I

# Then open http://<PI_IP>:8000 in a browser on any device on the same network.

# Make sure I2C is enabled (sudo raspi-config → Interface Options → I2C) and the LSM6DSOX is wired to the I2C1 bus (SDA=GPIO2, SCL=GPIO3). You can verify the sensor is detected with
# i2cdetect -y 1 — it should show up at address 0x6A or 0x6B.
