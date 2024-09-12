#!/usr/bin/env python3

from time import sleep

from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveTank, speed_to_speedvalue, SpeedNativeUnits, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_4

tank = MoveTank(OUTPUT_A, OUTPUT_B)
cs_left = ColorSensor(INPUT_1)
cs_right = ColorSensor(INPUT_4)
enable = True

def follow_line(kp,ki,kd,speed):
        integral = 0.0
        last_error = 0.0
        derivative = 0.0
        speed = speed_to_speedvalue(speed)
        speed_native_units = speed.to_native_units(tank.left_motor)

        ##########
        skew = 0.8
        ##########
        
        while enable:
            error = cs_left.reflected_light_intensity - cs_right.reflected_light_intensity*skew
            integral = integral + error
            derivative = error - last_error
            last_error = error
            turn_native_units = (kp * error) + (ki * integral) + (kd * derivative)

            left_speed = SpeedNativeUnits(speed_native_units - turn_native_units)
            right_speed = SpeedNativeUnits(speed_native_units + turn_native_units)

            tank.on(left_speed, right_speed)

follow_line(14,0.0,0,SpeedPercent(70))