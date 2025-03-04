#!/usr/bin/python3
# coding=utf8

import time
from utils import Utils

# Imports from existing ArmPi code
import sys
sys.path.append('/home/pi/ArmPi/')
from ArmIK.ArmMoveIK import ArmIK
import HiwonderSDK.Board as Board


class Motion():
    """Class to move the arm to specified coordinates"""

    def __init__(self):

        # Arm instance
        self.arm = ArmIK()

        # Coordinates for keeping back
        self.block_xyz = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        'stack': (-15 + 1, -7 - 0.5, 1.5),
        }

        # Parameters
        self.gripper_close = 500

        # Utils instance
        self.ut = Utils()

        # Home position
        self.home_position()

    def home_position(self):
        """Move the arm to the home position"""

        Board.setBusServoPulse(1, self.gripper_close -150, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.arm.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(0.5)

    def move_to(self, xyz:tuple, move_time:int=500):
        """Move the arm to specific coordinates"""

        self.arm.setPitchRangeMoving((xyz), -90, -90, 0, move_time)
        time.sleep(2)

    def pick(self, xy:tuple, angle:float):
        """Pick up the box at a specific coordinate"""

        # Open the gripper
        Board.setBusServoPulse(1, self.gripper_close - 280, 500)
        # Rotate the gripper
        servo2_angle = self.ut.getAngle(xy, angle)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.8)

        # Lower the arm
        self.move_to(xyz=(xy[0], xy[1] + 0.5, 1.5), move_time=1000)

        # Close the gripper
        Board.setBusServoPulse(1, self.gripper_close, 500)
        time.sleep(0.8)

        # Lift the arm
        self.move_to(xyz=(xy[0], xy[1], 12), move_time=1000)

        # Rotate the gripper
        Board.setBusServoPulse(2, 500, 500)
        time.sleep(0.5)

    def place(self, color:str, z_offset:float=0.5):
        """Place the block at coordinates defined by the color

        Args:
            color (str): Color of the block for location
            z_offset (float): Add Offset in height from the block home position
        """

        # Default to red if color not in block_xyz
        if not color in self.block_xyz or color == '':
            color = 'red'

        # Move to top of the block home position
        self.move_to(xyz=(self.block_xyz[color][0], self.block_xyz[color][1], self.block_xyz[color][2] + 12))

        # Rotate the gripper
        servo2_angle = self.ut.getAngle(self.block_xyz[color], -90)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.5)

        # Place the block
        place_location = list(self.block_xyz[color])
        place_location[2] += z_offset
        self.move_to(xyz=(place_location), move_time=1000)

        # Open the gripper
        Board.setBusServoPulse(1, self.gripper_close - 200, 500)
        time.sleep(0.8)

        # Lift the arm up
        self.move_to(xyz=(self.block_xyz[color][0], self.block_xyz[color][1], 12), move_time=800)

    def pick_place(self, location:dict, z_offset:float=0.25):
        """Pick the box at a specific coordinate and place it at the home coordinates defined by the color

        Args:
            location (dict): Dictionary with color as key and a tuple of xy and angle as value
        """

        color = list(location.keys())[0]
        xy = location[color][0]
        angle = location[color][1]

        # Home position
        self.home_position()

        # Check if reachable
        result = self.arm.setPitchRangeMoving((xy[0], xy[1], 5), -90, -90, 0)

        if result == False:
            print(f"Box is unreachable at {xy}")
            # Break the function if the box is unreachable
            return

        # Move to top of the box
        self.move_to(xyz=(xy[0], xy[1] + 0.5, 10), move_time=1000)

        # Pick up the box
        self.pick(xy, angle)

        # Place the box
        self.place(color, z_offset)

        # Home position
        self.home_position()

        return

    def sort(self, locations:dict):
        """Sort the boxes"""

        # Pick and place each box
        for color in locations.keys():
            location = {color: locations[color]}
            self.pick_place(location)

        return

    def stack(self, locations:dict):
        """Stack the boxes"""

        z_offset = 0.25

        # Pick and place each box with increasing z_offset
        for color in locations.keys():
            location = {'stack': locations[color]}
            self.pick_place(location, z_offset)
            z_offset += 2.5

        return


if __name__ == '__main__':

    # Motion object
    mt = Motion()

    # Test parameters
    color = 'red'
    xy = (0.0, 20.0)
    angle = -45.0

    # Test pick_place
    mt.pick_place(color=color, xy=xy, angle=angle)