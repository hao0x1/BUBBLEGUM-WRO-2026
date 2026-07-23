# K230 production camera program

[`main.py`](main.py) is the program installed as `/sdcard/main.py` on the
Lushan Pi K230. It sends camera observations to the ESP32 at 115200 baud on
GPIO 5 (TX) and GPIO 6 (RX). The ESP32 controls steering, drive, lap counting,
and recovery.

Use this same K230 program for Open and Obstacle. Open does not require camera
observations. Obstacle uses visible red/green pillars; parking is disabled.
The final obstacle recovery revision changes ESP32 behavior and leaves this
camera program unchanged.

Deployment and optional color calibration are documented in
[`tools/k230/README.md`](../../tools/k230/README.md). The installer preserves the
board's saved `/sdcard/colorThresholds.cfg`; there is no generic calibration
file to copy over the car's measured thresholds.
