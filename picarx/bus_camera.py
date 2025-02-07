from picarx_new import Picarx
from vilib import Vilib
import time
import logging
import numpy as np
import cv2
import concurrent.futures
from readerwriterlock import rwlock

# logging configuration
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level = logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)



class Bus():
    def __init__(self):
        self.message = None
        self.lock = rwlock.RWLockWrite()

    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message

    def read(self):
        with self.lock.gen_rlock():
            message = self.message
        return message




class Sense():
    def __init__(self, px, sense_bus, sense_delay):
        self.px = px
        self.sense_bus = sense_bus
        self.sense_delay = sense_delay
        
        Vilib.camera_start()
        time.sleep(0.5)
        self.path = "picarx"
        self.image = "image"
        self.px.set_cam_tilt_angle(-45)
        #Vilib.display()
    
    
    def capture_image(self):
        while True:
            try:
                Vilib.capture_image(self.image, self.path)
                self.sense_bus.write(f'{self.path}/{self.image}')
            except:
                logging.debug("Failed to Capture Image")

            time.sleep(self.sense_delay)



class Interpret():
    def __init__(self, sense_bus, control_bus, sense_delay, control_delay, polarity = False):

        self.polarity = polarity #False - dark line, True - White line

        self.sense_bus = sense_bus
        self.control_bus = control_bus

        self.sense_delay = sense_delay
        self.control_delay = control_delay


        self.robot_location = 0
        self.thresh = 75
        self.colour = 255
        self.lock = rwlock.RWLockWrite()
    

    def line_status(self):
        while True:
            try:
                file_name = self.sense_bus.read()
                bnw_image = cv2.imread(f'{file_name}.jpg')
                bnw_image = cv2.cvtColor(bnw_image, cv2.COLOR_BGR2GRAY)
                bnw_image = bnw_image[350:425, :]
                _, img_width = bnw_image.shape
                img_width /= 2

                if self.polarity:
                    ret, mask = cv2.threshold(bnw_image, thresh = self.thresh, maxval=self.colour, type = cv2.THRESH_BINARY)
                else:
                    ret, mask = cv2.threshold(bnw_image, thresh = self.thresh, maxval=self.colour, type = cv2.THRESH_BINARY_INV)
                
                try:
                    contours, hierarchy  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    biggest_contour = max(contours, key=cv2.contourArea)
                    M = cv2.moments(biggest_contour)

                except:
                    logging.debug("Contour not found")
                    time.sleep(self.sense_delay)
                    continue 

                if M['m00'] != 0:
                    with self.lock.gen_wlock():
                        self.robot_location = (int(M['m10']/M['m00']) - img_width)/img_width

                time.sleep(self.sense_delay)
            except:
                logging.debug("No image found")


    def robot_position(self):
        while True:
            try:
                with self.lock.gen_rlock():
                    self.control_bus.write(self.robot_location)
            except:
                logging.debug("Could not find robot location")
            time.sleep(self.control_delay)



class Control():
        def __init__(self, px, control_bus, control_delay, k_p = 30, k_i = 0.0, k_d = 5.0, threshold = 0.1):
            self.px = px
            self.k_p = k_p # Proportional gain
            self.k_i = k_i  # Integral gain
            self.k_d = k_d  # Derivative gain

            self.error = 0.0
            self.angle = 0.0
            self.prev_error = 0.0 # Previous error for derivative calculation

            self.threshold = threshold
            self.control_bus = control_bus
            self.control_delay = control_delay

            self.last_time = time.time() # Store last timestamp for derivative calculation
            
    

        # PID control to adjust the steering of the picar
        def steer(self):
            while True:
                try:
                    car_position = self.control_bus.read()
                    current_time = time.time()
                    dt = current_time - self.last_time  # Time difference for derivative term
                    if dt == 0:  # Prevent division by zero
                        dt = 0.001


                    if abs(car_position) > self.threshold:
                        proportional = self.k_p * car_position 
                        self.error += car_position * dt 
                        integral = self.k_i * self.error
                        derivative = self.k_d * ((car_position - self.prev_error) / dt)

                        self.angle = proportional + integral + derivative # total steering angle

                        logging.debug(f'P: {proportional}, I: {integral}, D: {derivative}, Angle: {self.angle}')

                        self.px.set_dir_servo_angle(self.angle)
                        self.prev_error = car_position  # Store error for next derivative calculation
                        self.last_time = current_time

                        time.sleep(sense_delay)
                        continue

                    else:
                        self.angle = 0
                        self.px.set_dir_servo_angle(self.angle)
                    time.sleep(self.control_delay)
                except:
                    logging.debug("No steering provided")
                time.sleep(sense_delay)





if __name__ == "__main__":

    px = Picarx()
    
    sense_bus = Bus()
    control_bus = Bus()

    sense_delay = 0.1
    control_delay = 0.1


    sense = Sense(px, sense_bus, sense_delay)
    think = Interpret(sense_bus, control_bus, sense_delay, control_delay, polarity = False)
    control = Control(control_bus, control_delay, px, threshold = 0.05)
    time.sleep(2)
    sense.px.forward(30)


    # Concurrency stuff

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        eSensor = executor.submit(sense.capture_image)
        eInterpreter = executor.submit(think.line_status)
        eRobot = executor.submit(think.robot_position)
        eControl = executor.submit(control.steer)


    eInterpreter.result()
    eSensor.result()
    eControl.result()
    eRobot.result()
