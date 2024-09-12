#!/usr/bin/env python3

from time import sleep

from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveTank, SpeedPercent, follow_for_ms
from ev3dev2.sensor.lego import ColorSensor

tank = MoveTank(OUTPUT_A, OUTPUT_B)
tank.cs = ColorSensor()

tank.on_for_seconds(SpeedPercent(-50),SpeedPercent(-50),5)


# try:
#     # Follow the line for 4500ms
#     tank.follow_line(
#         kp=3.5, ki=0.0, kd=0.0,
#         speed=SpeedPercent(10),
#         follow_for=follow_for_ms,
#         off_line_count_max=10000,
#         ms=15000000
#     )
# except LineFollowErrorTooFast:
#     tank.stop()
#     raise