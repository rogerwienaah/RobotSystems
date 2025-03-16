#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import Camera
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from CameraCalibration.CalibrationConfig import *

class Perception():
    def __init__(self):
        
        self.possible_colour_values = {'red': (0, 0, 255),
                                  'blue': (255, 0, 0),
                                  'green': (0, 255, 0),
                                  'black': (0, 0, 0),
                                  'white': (255, 255, 255)}
        


        self.target_color = ('red', 'green', 'blue')
        self.camera = Camera.Camera()
        self.camera.camera_open()

        self.img_size = (640, 480)
        self.blur_kernal = (11, 11)
        self.filter_kernal = (6, 6)
        self.std_kernal = 11
        self.roi = ()
        self.best_contour = None
        self.best_contour_area = 0
        self.color_of_interest = None

        self.minimum_contour_thresh = 711
        self.last_x = 0
        self.last_y = 0
        self.color_to_number = {"red" : 1, "green" : 2, "blue" : 3}
        self.number_to_color = {1 : "red", 2 : "green", 3 : "blue"}

        self.seen_colours = []
        self.center_locations = []
        self.movement_change_thresh = 0.5
        self.previous_time = time.time()
        self.time_threshold = 1.0
        self.current_colour = "None"
        self.draw_colour = self.possible_colour_values['black']
        self.rotation_angle = 0
        self.color_range = color_range
        # self.square_length = 1.6


    def find_objects(self):
        while True:
            img = self.camera.frame

            if img is not None:
                processed_img = self.process_img(img)
                cv2.imshow('Frame', processed_img)
                key = cv2.waitKey(1)
                if key == 27:
                    break
            
        self.camera.camera_close()
        cv2.destroyAllWindows()

    def process_img(self, img):
        height, width = img.shape[:2]

        # Draw calibration + on image to ensure we know where we are
        cv2.line(img, (0, int(height / 2)), (width, int(height / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(width / 2), 0), (int(width / 2), height), (0, 0, 200), 1)

        #Re-size and find regions of interest
        reshape_img = cv2.resize(img, self.img_size, interpolation=cv2.INTER_NEAREST)
        reshape_img_blur = cv2.GaussianBlur(reshape_img, self.blur_kernal, self.std_kernal)
        # try:
        #     reshape_img_blur = getMaskROI(reshape_img_blur, self.roi, self.img_size)
        # except:
        #     print("No contour found")
        img_lab_color = cv2.cvtColor(reshape_img_blur, cv2.COLOR_BGR2LAB)

        self.process_region_of_interest(img_lab_color)

        if self.best_contour_area > self.minimum_contour_thresh:
            rect = cv2.minAreaRect(self.best_contour)
            box = np.int0(cv2.boxPoints(rect))

            self.roi = getROI(box)
            img_x, img_y = getCenter(rect, self.roi, self.img_size, square_length)
            # img_x, img_y = rect[0]
            
            # focus on only the storage area - half of fov
            if img_x < self.img_size[0] // 2:
                world_x, world_y = convertCoordinate(img_x, img_y, self.img_size)

                cv2.drawContours(img, [box], -1, self.possible_colour_values[self.color_of_interest], 2)
                cv2.putText(img, f'({world_x}, {world_y})', (min(box[0, 0], box[2, 0]), box[2, 1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                            self.possible_colour_values[self.color_of_interest], 1) 

                distance = math.sqrt((world_x - self.last_x)**2 + (world_y - self.last_y)**2)
                self.last_x, self.last_y = world_x, world_y

                if self.color_of_interest in self.color_to_number:
                    color_location = self.color_to_number[self.color_of_interest]
                else:
                    color_location = 0
                
                self.seen_colours.append(color_location)

                if distance < self.movement_change_thresh:
                    self.center_locations.extend((world_x, world_y))
                    self.check_timing(rect)
                else:
                    self.previous_time = time.time()
                    self.center_locations = []
            
                if len(self.seen_colours) == 3:
                    current_number = int(round(np.mean(np.array(self.seen_colours))))
                    if current_number in self.number_to_color:
                        self.current_colour = self.number_to_color[current_number]
                        self.draw_colour = self.possible_colour_values[self.current_colour]
                    else:
                        self.current_colour = 'None'
                        self.draw_colour = self.possible_colour_values['black']
                        
                    
                    self.seen_colours = []
            
        else:
            self.draw_colour = (0, 0, 0)
            self.current_colour = "None"

        cv2.putText(img, f'Colour: {self.current_colour}', (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_colour, 2)
        return img
            
    def check_timing(self, rect):
        if time.time() - self.previous_time > self.time_threshold:
            self.rotation_angle = rect[2]
            # average_x, average_y = np.mean(np.array(self.center_locations).reshape(len(self.center_locations)/2, 2), axis = 0)
            self.center_locations = []
            self.previous_time = time.time()

    def process_region_of_interest(self, img_lab_color):
        self.best_contour = None
        self.best_contour_area = 0
        self.color_of_interest = None
        
        for color in self.color_range:
            if color in self.target_color:
                color_mask = cv2.inRange(img_lab_color, self.color_range[color][0], self.color_range[color][1]) # Find all values within given color range we want to analyze
                cleaned_image = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, np.ones(self.filter_kernal, np.uint8))
                cleaned_image = cv2.morphologyEx(cleaned_image, cv2.MORPH_CLOSE, np.ones(self.filter_kernal, np.uint8))
                contours = cv2.findContours(cleaned_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2] # Only give us the contours, don't care about the image or hierarchy

                try:
                    largest_contour = max(contours, key=cv2.contourArea)
                    largest_contour_area = cv2.contourArea(largest_contour)

                    if largest_contour_area > self.best_contour_area:
                        self.best_contour_area = largest_contour_area
                        self.best_contour = largest_contour
                        self.color_of_interest = color
                except:
                    continue
                

if __name__ == "__main__":
    perception = Perception()
    perception.find_objects()