from picarx_new import Picarx
from vilib import Vilib
import time
import logging
import numpy as np
import cv2

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level = logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

class Sense():
    def __init__(self, camera = False):
        self.px = Picarx()
        self.reference = np.array(self.px.grayscale._reference)
        if camera:
            Vilib.camera_start()
            time.sleep(0.5)
            #Vilib.display()
            self.path = "picarx"
            self.image_name = "image"
            self.px.set_cam_tilt_angle(-30)
    
    def get_grayscale(self):
        return np.array(self.px.grayscale.read()) - self.reference
    
    def take_photo(self):
        logging.debug("Photo Taken")
        Vilib.take_photo(photo_name = self.image_name, path = self.path)
        time.sleep(0.1)

class Interpret():
    def __init__(self, range = [0, 3600], polarity = False):
        ''' Initialize Interpreter
        
        param range: Indicates range of acceptable light -> dark values
        type range: list[int, int]
        param polarity: False indicates light floor, dark line, True indicates dark floor, light line
        type polarity: Bool      
        '''
        self.low_range, self.high_range = range
        self.polarity = polarity
        self.robot_location = 0
        self.thresh = 75
        self.colour = 255
        self.img_start = 350
        self.img_cutoff = 425
    
    def line_location_grayscale(self, grayscale_values):
        if self.polarity:
            grayscale_values = [grayscale_value - min(grayscale_values) for grayscale_value in grayscale_values] 
        else:
            grayscale_values = [abs(grayscale_value - max(grayscale_values)) for grayscale_value in grayscale_values] 

        left, middle, right = grayscale_values
        logging.debug(f'MODIFIED - Left: {left}, Middle: {middle}, Right: {right}')

        try:
            if left > right:
                self.robot_location = (middle - left) / max(left, middle)
                if self.robot_location < 0:
                    self.robot_location = self.robot_location
                    return
                self.robot_location -= 1
                return
            self.robot_location = (middle-right)/max(middle, right)
            if self.robot_location < 0:
                self.robot_location = -1*self.robot_location
                return
            self.robot_location = 1-self.robot_location
            return
        except:
            logging.debug(f'Divide by zero error, continuing')

    def line_location_camera(self, path, image_name):
        gray_img = cv2.imread(f'{path}/{image_name}.jpg')
        gray_img = cv2.cvtColor(gray_img, cv2.COLOR_BGR2GRAY)
        gray_img = gray_img[self.img_start:self.img_cutoff, :]
        _, img_width = gray_img.shape
        img_width /= 2
        if self.polarity:
            _, mask = cv2.threshold(gray_img, thresh = self.thresh, maxval=self.colour, type = cv2.THRESH_BINARY)
        else:
            _, mask = cv2.threshold(gray_img, thresh = self.thresh, maxval=self.colour, type = cv2.THRESH_BINARY_INV)
        
        try:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
        except:
            logging.debug("NO CONTOUR FOUND")
            return 

        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            self.robot_location = (int(M['m10']/M['m00']) - img_width)/img_width
            cv2.circle(gray_img, (cx, cy), 5, (0, 0, 255), -1)
        cv2.imshow("Gray", gray_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def robot_position(self):
        logging.debug(f'Robot Location: {self.robot_location}')
        return self.robot_location

class Control():
        def __init__(self, k_p = 30, k_i = 0.0, threshold = 0.1):
            self.k_p = k_p
            self.k_i = k_i
            self.threshold = threshold
            self.error = 0.0
            self.angle = 0.0
    
        def steer(self, px, car_position):
            if abs(car_position) > self.threshold:
                self.error += car_position
                self.angle = self.k_p * car_position + self.error * self.k_i
                logging.debug(f'Steering Angle: {self.angle}')
                px.set_dir_servo_angle(self.angle)
                return self.angle
            self.angle = 0
            logging.debug(f'Steering Angle: {self.angle}')
            px.set_dir_servo_angle(self.angle)
            return self.angle

if __name__ == "__main__":
    method = 0
    while method != 1 and method != 2:
        method = int(input("Select 1 for grayscale or 2 for camera based line following: "))
    
    if method == 1:
        sense = Sense(camera=False)
        think = Interpret(polarity = False)
        control = Control(threshold = 0.1)
        time.sleep(2)
        sense.px.forward(30)
        while True:
            think.line_location_grayscale(sense.get_grayscale())
            robot_position = think.robot_position()
            control.steer(sense.px, robot_position)
    elif method == 2:
        sense = Sense(camera=True)
        think = Interpret(polarity = False)
        control = Control(threshold = 0.05)
        time.sleep(2)
        sense.px.forward(30) 
        while True:
            sense.take_photo()
            think.line_location_camera(sense.path, sense.image_name)
            robot_position = think.robot_position()
            control.steer(sense.px, robot_position)
