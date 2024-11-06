#!/usr/bin/env python3

from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor import INPUT_3
from ev3dev2.button import Button
from time import sleep


distance_sensor = UltrasonicSensor(INPUT_3)
btn = Button()



state = False
while (True):

    if btn.any():
        state = not state
    
    if state:
        print("A")
        print(distance_sensor.value())
    else:
        print("B")
        print(distance_sensor.distance_centimeters)

    sleep(0.1)