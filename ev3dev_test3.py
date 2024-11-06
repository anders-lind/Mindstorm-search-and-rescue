#!/usr/bin/env python3

from time import sleep
import numpy as np
import sys

from ev3dev2.motor import OUTPUT_A, OUTPUT_D, MoveTank, speed_to_speedvalue, SpeedNativeUnits, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.display import Display
from ev3dev2.button import Button
from ev3dev2.sensor.lego import UltrasonicSensor



##### Initialize #####
print("Initializing")
enable = True   

# Init
disp = Display()
tank = MoveTank(OUTPUT_D, OUTPUT_A)
cs_left = ColorSensor(INPUT_1)
cs_right = ColorSensor(INPUT_4)
distance_sensor = UltrasonicSensor(INPUT_3)
btn = Button()


# PID vars
errors = []
integral = 0.0
last_error = 0.0
speed = speed_to_speedvalue(100)
speed_native_units = -speed.to_native_units(tank.right_motor)

# PID gains
kp = 15
ki = 0.0
kd = 2.0

# Speed scaling gains
max_speed = 0.3
speed_gain = 2

# Init light sensor skew
if abs(cs_right.reflected_light_intensity) > 0.1:
    skew = cs_left.reflected_light_intensity/cs_right.reflected_light_intensity
else:
    skew = 1

print("Initialized")
##### Initialize #####


def main(): 
    

    input("Press enter to start")
    follow_line()

    while (True):
        i = input("What now? (Line follow: 'l',    New gains 'g',    Print: 'p',    Quit: 'q')")
        if i == 'q':
            return
        elif i == 'g':
            new_gains()
        elif i == 'p':
            print(errors)
        elif i == 'l':
            follow_line()



def new_gains():
    global kp, ki, kd, max_speed, speed_gain

    print("Enter new gains:")
    kp = float(input("P: "))
    ki = float(input("I: "))
    kd = float(input("D: "))
    max_speed = float(input("max speed: "))
    speed_gain = float(input("speed_gain: "))

    

def follow_line():
    print("Follow line")

    while enable:
        global integral, last_error, speed_native_units

        # Check input
        if btn.any():
            tank.off()
            return

        # Rotational speed
        error =  cs_right.reflected_light_intensity*skew - cs_left.reflected_light_intensity
        integral = integral + error
        derivative = error - last_error
        last_error = error
        turn_native_units = (kp * error) + (ki * integral) + (kd * derivative)

        # Linear speed
        speed_scaled = max_speed / ((error/speed_gain)**2 + 1) * speed_native_units
        if speed_scaled > 1050:
            print("WARNING: speed_scaled = " + str(speed_scaled) + "  ( > 1050)")
            speed_scaled = 1050
        if speed_scaled < -1050:
            print("WARNING: speed_scaled = " + str(speed_scaled) + "  ( < -1050)")
            speed_scaled = -1050
        

        # Send speed to wheels
        left_speed = SpeedNativeUnits(speed_scaled - turn_native_units)
        right_speed = SpeedNativeUnits(speed_scaled + turn_native_units)
        tank.on(left_speed, right_speed)

        # Display values
        disp.text_grid("Left sensor:", True, 0 , 1)
        disp.text_grid(str(cs_left.reflected_light_intensity), False, 0 , 2)
        disp.text_grid("Right sensor:", False, 13 , 1)
        disp.text_grid(str(cs_right.reflected_light_intensity), False, 13 , 2)
        disp.text_grid("Error:", False, 9 , 4)
        disp.text_grid(str(error), False, 9 , 5)
        disp.update()

        # Print
        errors.append(error)
        print(error)



main()
