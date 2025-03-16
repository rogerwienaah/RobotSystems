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
from dice_bot_perception import Perception

class Motion():
    def __init__(self, perception):
        
        #coordinates for placing objects
        self.tower_coordinates = (-15 + 0.5, 15-0.5, 20)
        
        #safe zone
        self.safe_zone = (15, 10, 20)
        
        self.perception = perception
        self.currently_moving = False
        self.sleep_divider = 1000
        self.sleep_time = 0.5
        self.gripper_closed = 600
        self.gripper_open = 150
        self.servo_1_id = 1
        self.servo_2_id = 2
        self.arm_kinematics = ArmIK()
        self.desired_approach_height_grasp = 5
        self.desired_final_height_grasp = 2  # grasp based on the thickness of the board
    
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
            # check for color and right - half plane of fov
            if self.perception.current_colour != "None" and self.perception.last_x < self.perception.img_size[0] // 2:
                current_colour = self.perception.current_colour
                self.set_led_colour(current_colour)
                
                #get coordinates and rotation of the object
                desired_x, desired_y, desired_angle = self.perception.last_x+2, self.perception.last_y, self.perception.rotation_angle
                print(desired_x, desired_y)
                
                #move arm to above object
                result = self.arm_kinematics.setPitchRangeMoving((desired_x, desired_y, self.desired_approach_height_grasp), -90, -90, 0)  

                if result:
                    time.sleep(result[2]/self.sleep_divider)
                    
                    # rotate gripper to match object orientation
                    block_rotation = getAngle(desired_x, desired_y, desired_angle)
                    Board.setBusServoPulse(self.servo_1_id, self.gripper_closed - self.gripper_open, self.gripper_closed)
                    Board.setBusServoPulse(self.servo_2_id, block_rotation, self.gripper_closed)
                    time.sleep(self.sleep_time)
                    
                    # lower gripper for final grasp
                    self.arm_kinematics.setPitchRangeMoving((desired_x, desired_y, self.desired_final_height_grasp), -90, -90, 0, 1000)
                    time.sleep(self.sleep_time)
                    
                    #close gripper for final grab of object
                    Board.setBusServoPulse(self.servo_1_id, self.gripper_closed, self.gripper_closed)
                    time.sleep(self.sleep_time)

                    Board.setBusServoPulse(self.servo_2_id, self.gripper_closed, self.gripper_closed)
                    
                    # lift object
                    self.arm_kinematics.setPitchRangeMoving((desired_x, desired_y, self.desired_approach_height_grasp), -90, -90, 0, 1000)
                    time.sleep(2*self.sleep_time)
                    
                    # move to safe zone
                    result = self.arm_kinematics.setPitchRangeMoving((self.safe_zone[0], self.safe_zone[1], self.safe_zone[2] + 5), -90, -90, 0)   
                    time.sleep(2)
                    
                    
                    #move arm to drop position - tower location
                    result = self.arm_kinematics.setPitchRangeMoving((self.tower_coordinates[0], self.tower_coordinates[1], self.tower_coordinates[2] + 5), -90, -90, 0)   
                    time.sleep(result[2]/self.sleep_divider)
                                    
                    block_rotation = getAngle(self.tower_coordinates[0], self.tower_coordinates[1], -90)
                    Board.setBusServoPulse(self.servo_2_id, block_rotation, self.gripper_closed)
                    time.sleep(self.sleep_time)

                    self.arm_kinematics.setPitchRangeMoving((self.tower_coordinates[0], self.tower_coordinates[1], self.tower_coordinates[2] + 5), -90, -90, 0, 500)
                    time.sleep(self.sleep_time)
                                        
                    #self.arm_kinematics.setPitchRangeMoving((self.tower_coordinates), -90, -90, 0, 1000)
                    #time.sleep(self.sleep_time)

                    Board.setBusServoPulse(self.servo_1_id, self.gripper_closed - self.gripper_open, self.gripper_closed)
                    time.sleep(self.sleep_time)

                    self.arm_kinematics.setPitchRangeMoving((self.tower_coordinates[0], self.tower_coordinates[1], self.tower_coordinates[2] + 5), -90, -90, 0, 800)
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