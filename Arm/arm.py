#neccessary imports
#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/ArmPi/')
import cv2
import math
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *



if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

"""
writing my own class for perception
#tasks
to identiy colored boxes based on images,
Bounding box around the colored boxes
label the boxes with associated color
get the coordinates of the boxes

"""

class Perception:
    
    #initializing the class
    def __init__(self, color):
        # self.color = color
        pass
    #other methods

    # Find the contour with the largest area
# Parameter: A list of contours to compare
    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # Iterate through all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # Only consider the largest contour if its area is greater than 300 to filter out noise
                    area_max_contour = c

        return area_max_contour, contour_area_max  # Return the largest contour
    

    # Set the color of the RGB light on the expansion board to match the target color being tracked
    def set_rgb(self, color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()



    

