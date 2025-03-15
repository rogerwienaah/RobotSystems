#!/usr/bin/python3
# coding=utf8

import cv2
import math
import numpy as np
from utils import Utils

# Imports from existing ArmPi code
import sys
sys.path.append('/home/pi/ArmPi/')
from Camera import Camera


class Perception():
    """Perception class to detect objects and return their coordinates and angles"""

    def __init__(self):

        # For color of bounding box
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

        # Color range to detect
        self.color_range = {
            'red': [(0, 151, 100), (255, 255, 255)],
            'green': [(0, 0, 0), (255, 115, 255)],
            'blue': [(0, 0, 0), (255, 255, 110)],
            'black': [(0, 0, 0), (56, 255, 255)],
            'white': [(193, 0, 0), (255, 250, 255)],
        }

        # Possible color check
        self._possible_color = ['red', 'green', 'blue']

        # Variables
        self.size = (640, 480)
        self.roi = ()
        self.image_center_distance = 20
        self.square_length = 1.6

        # Camera instance
        self.camera = Camera()
        self.camera.camera_open()

        # Utils instance
        self.ut = Utils()

    def getAreaMaxContour(self, contours):
        """Find contour with the largest area
        Args:
            contours: Contour list to compare
        """

        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = c

        return area_max_contour, contour_area_max

    def process_image(self, img):
        """Process the image and return detected object coordinates
        Args:
            img: Image to process

        Returns:
            img: Image with detected objects in bounding boxes
            location: Dictionary with key-color detected and value-object coordinates and angles
        """

        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

        # Convert image to LAB color space
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        area_max = 0
        areaMaxContour = 0

        # Dictionary to store the coordinates and angles of the detected objects
        location = {}

        # Loop through all possible colors
        for i in self._possible_color:
            # Mask the image
            frame_mask = cv2.inRange(frame_lab, self.color_range[i][0], self.color_range[i][1])
            # Opening and closing operations to remove noise
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((4, 4), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((4, 4), np.uint8))
            # Find contours
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            # Find the contour with the largest area
            areaMaxContour, area_max = self.getAreaMaxContour(contours)

            if area_max > 711:
                rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(rect))

                # Get ROI
                self.roi = self.ut.getROI(box)

                # Get the center of the object
                img_centerx, img_centery = self.ut.getCenter(rect, self.roi, self.size, self.square_length)
                # Convert the center of the object to the world coordinate
                center_x, center_y = self.ut.convertCoordinate(img_centerx, img_centery, self.size, self.image_center_distance)
                angle = rect[2]

                location[i] = [(center_x, center_y), angle]

                cv2.drawContours(img, [box], -1, self.range_rgb[i], 2)
                # Draw the center of the object
                cv2.putText(img, '(' + str(center_x) + ',' + str(center_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[i], 1)

        return img, location

    def run(self):
        """Main run loop"""

        # Continuously process the image
        while True:
            img = self.camera.frame
            if img is not None:
                frame = img.copy()
                frame_processed, location = self.process_image(frame)

                cv2.imshow('Frame', frame_processed)
                key = cv2.waitKey(1)

                if key == 27:
                    # Press 'ESC' to exit
                    break

        # Close the camera and destroy the windows
        self.camera.camera_close()
        cv2.destroyAllWindows()


if __name__ == '__main__':

    # Perception module
    ct = Perception()
    # Run with detecting all possible colors
    ct.run()