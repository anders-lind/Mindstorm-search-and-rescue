#!/usr/bin/env python3
from ev3dev2.sound import Sound
from ev3dev2.display import Display
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
from ev3dev2.motor import OUTPUT_A, OUTPUT_D, OUTPUT_B, MoveDifferential, MoveTank, speed_to_speedvalue, SpeedNativeUnits, Motor, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.button import Button
from ev3dev2.wheel import EV3Tire
from time import sleep




class CompetetiveBehavior():
    def __init__(self):

        self.disp = Display()
        self.btn = Button()
        self.spkr = Sound()

        self.enabled = False
        self.errors = []
        self.left_light = []
        self.right_light = []
        
        # Counters
        self.tick_counter = 0
        self.off_line_counter = 0
        self.tighten_grip_counter = 0

        # Activation score parameters
        self.activation_scores = []
        self.has_can = False
        self.striped_line_threshold = 11
        self.off_line_threshold = 11
        self.fetch_can_score_multiplier = 4.0
        self.tighten_grip_score_multiplier = 0.25

        # Motors
        self.tank = MoveTank(OUTPUT_A, OUTPUT_D)
        self.diff = MoveDifferential(OUTPUT_A, OUTPUT_D, EV3Tire, 120)
        self.diff.gyro = GyroSensor(INPUT_3)
        self.gripper = Motor(OUTPUT_B)

        # Sensors
        self.cs_left = ColorSensor(INPUT_1)
        self.cs_right = ColorSensor(INPUT_4)
        self.skew = 0.8

        # PID
        self.errors = []
        self.integral = 0.0
        self.last_error = 0.0
        self.speed = speed_to_speedvalue(100)
        self.speed_native_units = -self.speed.to_native_units(self.tank.right_motor)
        # PID gains
        self.kp = 45
        self.ki = 0.0
        self.kd = 7.0
        
        # Speed scaling gains
        self.max_speed = 0.2
        self.speed_gain = 6
        self.stripe_speed_scale = 0.25

        # Fetch cans vars
        self.fetch_move_back = 100
        self.fetch_move_towards_can = 300
        self.fetch_move_towards_line = 200

        # Init display
        self.disp.text_grid("Error:", False, 9 , 4)
        self.disp.update()



    def run(self):
        while True:
            # self.fetch_can()

            self.tick_counter += 1

            # Calculate all scores
            self.activation_scores = []
            self.activation_scores.append(self.activation_follow_line())
            self.activation_scores.append(self.activation_fetch_can())
            self.activation_scores.append(self.activation_tigthen_grip())

            # Debug prints and more
            self.debug()

            # Find highest score
            max_score = max(self.activation_scores)
            max_index = self.activation_scores.index(max_score)


            # Run behavior
            if max_index == 0:
                self.follow_line()
            elif max_index == 1:
                # self.fetch_can()
                if self.tick_counter < 600:
                    print("Fake fetch can")
                    self.follow_line()
                    self.off_line_counter = 0
                else:
                    self.fetch_can()
            elif max_index == 2:
                self.tighten_grip()
    


    def debug(self):
        # Print scores
        if self.tick_counter % 2 == 0:
            print(self.tick_counter)
            print("Follow line score: ", self.activation_scores[0],"    Fetch can score:   ", self.activation_scores[1], "    Tigthen grip score:", self.activation_scores[2])
            print()
        
        # Display values
        error = self.cs_left.reflected_light_intensity - (self.cs_right.reflected_light_intensity + self.skew)
        self.disp.text_grid("Left sensor:", True, 0 , 1)
        self.disp.text_grid(str(self.cs_left.reflected_light_intensity), False, 0 , 2)
        self.disp.text_grid("Right sensor:", False, 13 , 1)
        self.disp.text_grid(str(self.cs_right.reflected_light_intensity), False, 13 , 2)
        self.disp.text_grid("Error:", False, 9 , 4)
        self.disp.text_grid(str(error), False, 9 , 5)
        self.disp.text_grid("Follow line: ", False, 0 , 8)
        self.disp.text_grid(str(self.activation_scores[0]), False, 0 , 9)
        self.disp.text_grid("Fetch can:   ", False, 13 , 8)
        self.disp.text_grid(str(self.activation_scores[1]), False, 13 , 9)
        self.disp.text_grid("Tigthen grip:", False, 9 , 10)
        self.disp.text_grid(str(self.activation_scores[2]), False, 9 , 11)
        self.disp.update()

        # Debug menu
        if self.btn.enter:
            self.tank.off()
            self.diff.off()
            
            while True:
                i = input("What now? (Run: 'r',    New gains 'g',    New diff 'd'    Init gripper 'i',    Cange fetch 'f',    Test gripper 't'/'tc')")
                if i == 'r':
                    break
                elif i == 'g':
                    self.new_gains()
                elif i == 'i':
                    self.gripper_manual_init()
                elif i == 'd':
                    self.new_diff()
                elif i == 't':
                    self.test_gripper()
                elif i == 'tc':
                    self.test_gripper(True)
                elif i == "f":
                    self.change_fetch_gains()


            
    def change_fetch_gains(self):
        fetch_move_back = input("Move back: ")
        if fetch_move_back != "":
            self.fetch_move_back = float(fetch_move_back)
        fetch_move_towards_can = input("Move towards can: ")
        if fetch_move_towards_can != "":
            self.fetch_move_towards_can = float(fetch_move_towards_can)
        fetch_move_towards_line = input("Move towards line: ")
        if fetch_move_towards_line != "":
            self.fetch_move_towards_line = float(fetch_move_towards_line)


    def gripper_manual_init(self):
        print("Gripper manual init: Right button to close, left button to open")
        while True:
            if self.btn.right:
                self.gripper.on_for_seconds(SpeedPercent(50), 1)
            elif self.btn.left:
                self.gripper.on_for_seconds(SpeedPercent(-30), 1)
            elif self.btn.up:
                return
    


    def open_gripper(self):
        self.gripper.on_for_degrees(-10, 960)
        # self.gripper.on_for_degreexs(-10, 960*2)

    def grip_can(self):
        # self.gripper.on_for_degrees(10, 760)
        self.gripper.on_for_degrees(10, 760*2)
        # self.gripper.on_for_degrees(10, 760*4)


    


    def new_gains(self):
        print("Enter new gains:")
        p = input("P: ")
        if p != "":
            self.kp = float(p)
        i = input("I: ")
        if i != "":
            self.ki = float(i)
        d = input("D: ")
        if d != "":
            self.kd = float(d)
        max_speed = input("Max speed: ")
        if max_speed != "":
            self.max_speed = float(max_speed)
        speed_gain = input("Speed gain: ")
        if speed_gain != "":
            self.speed_gain = float(speed_gain)
        stripe_speed_scale = input("Stripe_speed_scale: ")
        if stripe_speed_scale != "":
            self.stripe_speed_scale = stripe_speed_scale


    def new_diff(self):
        self.diff = MoveDifferential(OUTPUT_A, OUTPUT_D, EV3Tire, float(input("wheel base [mm]: ")))



    def activation_follow_line(self) -> float:
        activation_score = 100
        return activation_score


    def activation_fetch_can(self) -> float:
        if self.has_can:
            return -1
        elif(self.off_line_counter < 0):
            self.off_line_counter = 0
        elif((self.cs_left.reflected_light_intensity >= self.off_line_threshold) and (self.cs_right.reflected_light_intensity >= self.off_line_threshold)):
        # elif (self.cs_left.reflected_light_intensity + self.cs_right.reflected_light_intensity >= 23)
            self.off_line_counter += 1
        else:
            self.off_line_counter -= 5

        activation_score = self.off_line_counter * self.fetch_can_score_multiplier
        return activation_score
    

    def activation_tigthen_grip(self) -> float:
        self.tighten_grip_counter += 1

        activation_score = self.tighten_grip_counter * self.tighten_grip_score_multiplier
        return activation_score


    def tighten_grip(self):
        print("Tighten grip")
        self.tighten_grip_counter = 0
        self.gripper.on_for_seconds(50, 0.1, block=False)
        
        

    def follow_line(self) -> None:
        # Rotational speed
        error = self.cs_left.reflected_light_intensity - (self.cs_right.reflected_light_intensity + self.skew)
        self.integral = self.integral + error
        derivative = error - self.last_error
        turn_speed = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error
        self.errors.append(error)
        self.left_light.append(self.cs_left.reflected_light_intensity)
        self.right_light.append(self.cs_right.reflected_light_intensity)

        # Linear speed
        speed_scaled = self.max_speed / ((error/self.speed_gain)**2 + 1)
        speed_scaled *=  self.speed_native_units
        
        # Slow down at striped lines
        if self.cs_left.reflected_light_intensity >= self.striped_line_threshold and ((self.cs_right.reflected_light_intensity + self.skew) >= self.striped_line_threshold):
            speed_scaled = speed_scaled * self.stripe_speed_scale

        # Convert to native unitsSlow
        left_speed = SpeedNativeUnits(speed_scaled - turn_speed)
        right_speed = SpeedNativeUnits(speed_scaled + turn_speed)

        # Clamp speed values
        if abs(left_speed.native_counts) > 1050 or abs(right_speed.native_counts) > 1050:
            print("WARNING: speed_scaled > 1050")
            if left_speed.native_counts > 1050:
                left_speed.native_counts = 1050
            elif left_speed.native_counts < -1050:
                left_speed.native_counts = -1050
            if right_speed.native_counts > 1050:
                right_speed.native_counts = 1050
            elif right_speed.native_counts < -1050:
                right_speed.native_counts = -1050

        # Send speed to wheels
        self.tank.on(left_speed, right_speed)

        # # Display values
        # self.disp.text_grid("Left sensor:", True, 0 , 1)
        # self.disp.text_grid(str(self.cs_left.reflected_light_intensity), False, 0 , 2)
        # self.disp.text_grid("Right sensor:", False, 13 , 1)
        # self.disp.text_grid(str(self.cs_right.reflected_light_intensity), False, 13 , 2)
        # self.disp.text_grid("Error:", False, 9 , 4)
        # self.disp.text_grid(str(error), False, 9 , 5)
        # self.disp.update()



    def fetch_can(self) -> None:
        print("fetch_can")
        self.off_line_counter = -1
        self.has_can = True

        # print("1")
        self.diff.on_for_distance(10, self.fetch_move_back)
        # sleep(2)
        # print("2")
        self.diff.turn_degrees(10, 180, use_gyro=False)
        # sleep(3)
        # print("3")
        self.open_gripper()
        # sleep(2)
        # print("4")
        self.diff.on_for_distance(10, self.fetch_move_towards_can)
        # sleep(3)
        # print("5")
        self.grip_can()
        # sleep(10)
        # print("6")
        self.diff.on_for_distance(-10, self.fetch_move_towards_line)
        # sleep(2)



    def test_gripper(self, close=False) -> None:
        if close:
            self.gripper.on_for_degrees(10, 40)
        else:
            self.gripper.on_for_degrees(-10, 40)




cb = CompetetiveBehavior()
cb.spkr.play_tone(432, 0.25, volume=40)
cb.gripper_manual_init()
cb.run()