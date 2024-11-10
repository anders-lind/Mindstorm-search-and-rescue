#!/usr/bin/env python3

from time import sleep
import numpy as np
import sys

from ev3dev2.motor import OUTPUT_A, OUTPUT_D, MoveDifferential, MoveTank, speed_to_speedvalue, SpeedNativeUnits, Motor
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.display import Display
from ev3dev2.button import Button
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sound import Sound
from ev3dev2.wheel import EV3Tire


 
##### Initialize #####
print("Initializing")
enable = True   

# Init
disp = Display()
tank = MoveTank(OUTPUT_D, OUTPUT_A)
diff = MoveDifferential(OUTPUT_D, OUTPUT_A, EV3Tire, 120)
grip = Motor()
cs_left = ColorSensor(INPUT_1)
cs_right = ColorSensor(INPUT_4)
distance_sensor = UltrasonicSensor(INPUT_3)
btn = Button()
spkr = Sound()
spkr.set_volume(50)


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
    while (True):
        i = input("What now? (Line follow: 'l',    detect can: 'd'    New gains 'g',    New diff 'w'    Print: 'p',    Sound: 's',    Reset gripper: 'r'    Test gripper: 't'    Quit: 'q')")
        if i == 'q':
            return
        elif i == 'g':
            new_gains()
        elif i == 'p':
            print(errors)
        elif i == 'l':
            follow_line()
        elif i == 'd':
            detect_can()
        elif i == 'w':
            new_diff()
        elif i == 's':
            sound()
        elif i == 't':
            test_gripper()
        elif i == 'r':
            reset_gripper()



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


def new_gains():
    global kp, ki, kd, max_speed, speed_gain

    print("Enter new gains:")
    kp = float(input("P: "))
    ki = float(input("I: "))
    kd = float(input("D: "))
    max_speed = float(input("max speed: "))
    speed_gain = float(input("speed_gain: "))

def new_diff():
    global diff
    diff = MoveDifferential(OUTPUT_D, OUTPUT_A, EV3Tire, float(input("wheel base [mm]: ")))

def test_gripper():
    grip.on_for_degrees(10, 360*float(input("No. of rotations: ")))


def detect_can():
    n = 20
    distances = []

    # turn and save distances
    diff.turn_degrees(10, -90)
    distances.append(distance_sensor.distance_centimeters)
    for i in range(n):
        diff.turn_degrees(10, 180/n)
        distances.append(distance_sensor.distance_centimeters)
        print(distance_sensor.distance_centimeters)
    

    # Find angle with lowest distance
    lowest_distance_mm = 300
    lowest_index = -1
    for i in range(int(n)):
        if distances[i] < lowest_distance_mm:
            lowest_distance_mm = distances[i]
            lowest_index = i
    

    print("lowest distance at", lowest_index, lowest_distance_mm)
    print("Turn:", -180/n*(n-lowest_index))
    print("n = :", n, "i = ",lowest_index)

    # Go to angle with lowest distance
    diff.turn_degrees(10, -180/n*(n-lowest_index))

    # Open gripper
    grip.on_for_degrees(10, 360*-2)

    # Go to can
    diff.on_for_distance(10, lowest_distance_mm*10)
    # tank.on_for_seconds(10, 10, 1)

    # Close gripper
    grip.on_for_degrees(10, 360*4)

    print()


def reset_gripper():
    grip.on_for_degrees(10, 360*-2)
    


def sound():
    spkr.set_volume(int(input("Volume: ")))
    spkr.play_song((
    ('D4', 'e3'),
    ('D4', 'e3'),
    ('D4', 'e3'),
    ('G4', 'h'),
    ('D5', 'h') )
    , )

main()
