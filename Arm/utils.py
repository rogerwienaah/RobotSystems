#!/usr/bin/python3
# coding=utf8

import math
import numpy as np


class Utils():
    """Util functions for the arm"""

    def __init__(self):
        # Parameters from the camera calibration
        self.map_param_path = '/home/pi/ArmPi/CameraCalibration/map_param.npz'
        self.param_data = np.load(self.map_param_path)
        self.map_param_ = self.param_data['map_param']

    def getAngle(self, xy, angle):
        theta6 = round(math.degrees(math.atan2(abs(xy[0]), abs(xy[1]))), 1)
        angle = abs(angle)

        if xy[0] < 0:
            if xy[1] < 0:
                angle1 = -(90 + theta6 - angle)
            else:
                angle1 = theta6 - angle
        else:
            if xy[1] < 0:
                angle1 = theta6 + angle
            else:
                angle1 = 90 - theta6 - angle

        if angle1 > 0:
            angle2 = angle1 - 90
        else:
            angle2 = angle1 + 90

        if abs(angle1) < abs(angle2):
            servo_angle = int(500 + round(angle1 * 1000 / 240))
        else:
            servo_angle = int(500 + round(angle2 * 1000 / 240))

        return servo_angle

    def getROI(self, box):
        x_min = min(box[0, 0], box[1, 0], box[2, 0], box[3, 0])
        x_max = max(box[0, 0], box[1, 0], box[2, 0], box[3, 0])
        y_min = min(box[0, 1], box[1, 1], box[2, 1], box[3, 1])
        y_max = max(box[0, 1], box[1, 1], box[2, 1], box[3, 1])

        return (x_min, x_max, y_min, y_max)

    def leMap(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def world2pixel(self, l, size):
        l_ = round(l/self.map_param_, 2)
        l_ = self.leMap(l_, 0, 640, 0, size[0])

        return l_

    def convertCoordinate(self, x, y, size, image_center_distance=20.0):
        x = self.leMap(x, 0, size[0], 0, 640)
        x = x - 320
        x_ = round(x * self.map_param_, 2)

        y = self.leMap(y, 0, size[1], 0, 480)
        y = 240 - y
        y_ = round(y * self.map_param_ + image_center_distance, 2)

        return x_, y_

    def getCenter(self, rect, roi, size, square_length):
        x_min, x_max, y_min, y_max = roi
        # Select vertex closest to the center of the image based on the center of the object
        if rect[0][0] >= size[0]/2:
            x = x_max
        else:
            x = x_min
        if rect[0][1] >= size[1]/2:
            y = y_max
        else:
            y = y_min

        # Calculate diagonal length
        square_l = square_length/math.cos(math.pi/4)

        # Convert length to pixel length
        square_l = self.world2pixel(square_l, size)

        # Calculate center based on rotation angle of the object
        dx = abs(math.cos(math.radians(45 - abs(rect[2]))))
        dy = abs(math.sin(math.radians(45 + abs(rect[2]))))

        if rect[0][0] >= size[0] / 2:
            x = round(x - (square_l/2) * dx, 2)
        else:
            x = round(x + (square_l/2) * dx, 2)
        if rect[0][1] >= size[1] / 2:
            y = round(y - (square_l/2) * dy, 2)
        else:
            y = round(y + (square_l/2) * dy, 2)

        return  x, y