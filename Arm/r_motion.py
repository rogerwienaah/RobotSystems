#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
from upgrade_perception import Perception

class Motion():
    def __init__(self, perception):
        self.colour_coordinates = {'red' : (-15 + 0.5, 12 - 0.5, 1.5),
        'green' : (-15 + 0.5, 6 - 0.5,  1.5),
        'blue' : (-15 + 0.5, 0 - 0.5,  1.5)}
        self.perception = perception
        self.currently_moving = False
        self.sleep_divider = 1000
        self.sleep_time = 0.5
        self.gripper_closed = 500
        self.gripper_open = 280
        self.servo_1_id = 1
        self.servo_2_id = 2
        self.arm_kinematics = ArmIK()
        self.desired_approach_height_grasp = 7
        self.desired_final_height_grasp = 1.0
    
    def set_led_colour(self, colour):
        if colour == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif colour == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif colour == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()
    
    def move_home(self):
        Board.setBusServoPulse(1, self.gripper_closed - 50, self.gripper_open)
        Board.setBusServoPulse(2, self.gripper_closed, self.gripper_closed)
        self.arm_kinematics.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(self.sleep_time)
    
    def move_arm(self):
        while True:
            if self.perception.current_colour != "None":
                current_colour = self.perception.current_colour
                self.set_led_colour(current_colour)

                desired_x, desired_y, desired_angle = self.perception.last_x, self.perception.last_y, self.perception.rotation_angle
                result = self.arm_kinematics.setPitchRangeMoving((desired_x, desired_y, self.desired_approach_height_grasp), -90, -90, 0)  

                if result:
                    time.sleep(result[2]/self.sleep_divider)

                    block_rotation = getAngle(desired_x, desired_y, desired_angle)
                    Board.setBusServoPulse(self.servo_1_id, self.gripper_closed - self.gripper_open, self.gripper_closed)
                    Board.setBusServoPulse(self.servo_2_id, block_rotation, self.gripper_closed)
                    time.sleep(self.sleep_time)

                    self.arm_kinematics.setPitchRangeMoving((desired_x, desired_y, self.desired_final_height_grasp), -90, -90, 0, 1000)
                    time.sleep(self.sleep_time)

                    Board.setBusServoPulse(self.servo_1_id, self.gripper_closed, self.gripper_closed)
                    time.sleep(self.sleep_time)

                    Board.setBusServoPulse(self.servo_2_id, self.gripper_closed, self.gripper_closed)
                    self.arm_kinematics.setPitchRangeMoving((desired_x, desired_y, self.desired_approach_height_grasp), -90, -90, 0, 1000)
                    time.sleep(2*self.sleep_time)

                    result = self.arm_kinematics.setPitchRangeMoving((self.colour_coordinates[current_colour][0], self.colour_coordinates[current_colour][1], 12), -90, -90, 0)   
                    time.sleep(result[2]/self.sleep_divider)
                                    
                    block_rotation = getAngle(self.colour_coordinates[current_colour][0], self.colour_coordinates[current_colour][1], -90)
                    Board.setBusServoPulse(self.servo_2_id, block_rotation, self.gripper_closed)
                    time.sleep(self.sleep_time)

                    self.arm_kinematics.setPitchRangeMoving((self.colour_coordinates[current_colour][0], self.colour_coordinates[current_colour][1], self.colour_coordinates[current_colour][2] + 3), -90, -90, 0, 500)
                    time.sleep(self.sleep_time)
                                        
                    self.arm_kinematics.setPitchRangeMoving((self.colour_coordinates[current_colour]), -90, -90, 0, 1000)
                    time.sleep(self.sleep_time)

                    Board.setBusServoPulse(1, self.gripper_closed - self.gripper_open, self.gripper_closed)
                    time.sleep(self.sleep_time)

                    self.arm_kinematics.setPitchRangeMoving((self.colour_coordinates[current_colour][0], self.colour_coordinates[current_colour][1], 12), -90, -90, 0, 800)
                    time.sleep(self.sleep_time)

                    self.move_home()

                    current_colour = 'None'
                    self.set_led_colour(current_colour)
                    time.sleep(3*self.sleep_time)

if __name__ == "__main__":
    perception = Perception()
    motion = Motion(perception)

    t1 = threading.Thread(target=perception.find_objects)
    t2 = threading.Thread(target=motion.move_arm)

    t1.start()
    t2.start()