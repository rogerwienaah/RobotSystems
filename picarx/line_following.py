from picarx_new import Picarx
from vilib import Vilib
import time
import logging
import numpy as np
import cv2

# logging configuration
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level = logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)


class Sense():
    def __init__(self, camera = False):
        self.px = Picarx()
        self.reference = np.array(self.px.grayscale._reference)

        # Check for camera choice
        if camera:
            Vilib.camera_start()
            time.sleep(0.5)
            self.path = "picarx"
            self.image_name = "img"
            self.px.set_cam_tilt_angle(-30)
            #Vilib.display()
    
    def get_grayscale_data(self):
        return np.array(self.px.grayscale.read()) - self.reference
    
    def capture_image(self):
        Vilib.take_photo(self.image_name, self.path)
        logging.debug("took image")
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
    
    def get_line_status_grayscale(self, grayscale_values):

        # if black line
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
        # get line location and return robot position wrt line

        gray_img = cv2.imread(f'{path}/{image_name}.jpg')
        gray_img = cv2.cvtColor(gray_img, cv2.COLOR_BGR2GRAY)
        gray_img = gray_img[350:425, :]
        _, img_width = gray_img.shape
        img_width /= 2

        # white line
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
            # cx = int(M['m10']/M['m00'])
            # cy = int(M['m01']/M['m00'])
            self.robot_location = (int(M['m10']/M['m00']) - img_width)/img_width
            # cv2.circle(gray_img, (cx, cy), 5, (0, 0, 255), -1)
        # cv2.imshow("Gray", gray_img)
        # cv2.waitKey(0)
        #cv2.destroyAllWindows()

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
        

"""
How to Tune the PID Controller
Step 1: Start with P only
Set k_i = 0 and k_d = 0.
Increase k_p until the robot follows the line but oscillates slightly.
Step 2: Add D to Stabilize
Increase k_d until oscillations reduce.
If the robot becomes too slow to react, reduce k_d.
Step 3: Add I to Correct Drift
Slowly increase k_i to correct long-term drift.
If the robot starts overcorrecting, reduce k_i.

"""


# class Control():
#     def __init__(self, k_p=30, k_i=0.0, k_d=5.0, threshold=0.1):
#         self.k_p = k_p  # Proportional gain
#         self.k_i = k_i  # Integral gain
#         self.k_d = k_d  # Derivative gain
#         self.threshold = threshold  # Dead zone threshold

#         self.error = 0.0  # Accumulated integral error
#         self.prev_error = 0.0  # Previous error for derivative calculation
#         self.angle = 0.0  # Steering angle
#         self.last_time = time.time()  # Store last timestamp for derivative calculation

#     def steer(self, px, car_position):
#         current_time = time.time()
#         dt = current_time - self.last_time  # Time difference for derivative term

#         if dt == 0:  # Prevent division by zero
#             dt = 0.001

#         if abs(car_position) > self.threshold:
#             proportional = self.k_p * car_position  # P term
#             self.error += car_position * dt  # Accumulate integral error
#             integral = self.k_i * self.error  # I term
#             derivative = self.k_d * ((car_position - self.prev_error) / dt)  # D term

#             self.angle = proportional + integral + derivative  # Compute total steering angle

#             logging.debug(f'P: {proportional}, I: {integral}, D: {derivative}, Angle: {self.angle}')

#             px.set_dir_servo_angle(self.angle)  # Apply steering correction

#             self.prev_error = car_position  # Store error for next derivative calculation
#             self.last_time = current_time  # Update last time

#             return self.angle

#         self.angle = 0
#         logging.debug(f'Steering Angle: {self.angle}')
#         px.set_dir_servo_angle(self.angle)  # Keep wheels straight if error is small
#         return self.angle
    


if __name__ == "__main__":
    choice = 0
    while choice != 1 and choice != 2:
        choice = int(input("Select 1 for grayscale or 2 for camera based line following: "))
    
    # Grayscale line following
    if choice == 1:
        sense = Sense(camera=False)
        think = Interpret(polarity = True)
        control = Control(threshold = 0.1)
        time.sleep(2)
        sense.px.forward(30)
        while True:
            think.get_line_status_grayscale(sense.get_grayscale_data())
            robot_position = think.robot_position()
            control.steer(sense.px, robot_position)

    # Camera line following 
    elif choice == 2:
        sense = Sense(camera=True)
        think = Interpret(polarity = False)
        control = Control(threshold = 0.05)
        time.sleep(2)
        sense.px.forward(30) 
        while True:
            sense.capture_image()
            think.line_location_camera(sense.path, sense.image_name)
            robot_position = think.robot_position()
            control.steer(sense.px, robot_position)
