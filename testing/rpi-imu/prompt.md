Use your bash and file system tools to autonomously build a high-frequency, real-time web dashboard for an I2C LSM6DSOX IMU sensor on my Raspberry Pi 5. Please execute the following steps from start to finish:
 * Environment Setup: Use your bash tool to check if a Python virtual environment named .venv exists in the current directory. If it does not, run python3 -m venv .venv to create it.
 * Dependencies: Use your bash tool to install the necessary hardware and web libraries by executing .venv/bin/pip install Adafruit-Blinka adafruit-circuitpython-lsm6ds fastapi uvicorn websockets.
 * Backend (app.py): Use your file creation tools to write a complete app.py file that:
   * Sets up a FastAPI application.
   * Initializes the I2C bus (board.I2C()) and the LSM6DSOX sensor from adafruit_lsm6ds.lsm6dsox.
   * Mounts a root HTTP route (/) to serve an index.html file via FileResponse.
   * Creates a WebSocket endpoint (/ws) containing an async while True loop.
   * Inside the loop, reads the acceleration and gyro tuples, formats them into a JSON object, and sends them over the WebSocket. Include an asyncio.sleep(0.01) delay to achieve a stable ~100Hz polling rate—the highest frequency the hardware and browser can comfortably handle without bottlenecking the Pi's CPU.
 * Frontend (index.html): Use your file creation tools to write an index.html file in the same directory that:
   * Imports a high-performance, lightweight charting library via CDN (like SmoothieCharts or uPlot, which are optimized for real-time data).
   * Contains two separate charting areas: one for Acceleration (X, Y, Z) and one for Gyroscope (X, Y, Z).
   * Uses JavaScript to open a connection to the /ws WebSocket.
   * Parses the incoming JSON stream and dynamically appends the data to the charts so they smoothly scroll from right to left in real-time.
 * Final Output: Once you have successfully executed the bash commands and written the files, output the exact command I need to run to start the server (e.g., .venv/bin/python -m uvicorn app:app --host 0.0.0.0 --port 8000) and the command to find my Pi's IP address (hostname -I) so I can view the dashboard from another machine on my network. Do not run the server yourself; just provide the instructions.
