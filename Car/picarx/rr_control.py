from picarx_new import Picarx
try:
    from vilib import Vilib
except:
    pass
import time
import logging
import numpy as np
import cv2
import rossros as ros


logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level = logging.INFO, datefmt="%H:%M:%S")
# logging.getLogger().setLevel(logging.DEBUG)

class Sense():
    def __init__(self, px, camera = False):
        self.px = px
        self.reference = np.array(self.px.grayscale._reference)
        self.ultrasonic_distance = 0
        self.px.get_distance()
        if camera:
            Vilib.camera_start()
            time.sleep(0.5)
            #Vilib.display()
            self.path = "picarx"
            self.image_name = "image"
            self.px.set_cam_tilt_angle(-30)
    
    def get_grayscale_from_hardware(self):
        return np.array(self.px.grayscale.read()) - self.reference
    
    def get_ultrasonic_from_hardware(self):
        return self.px.get_distance()
    
    def take_photo(self):
        try:
            Vilib.take_photo(photo_name = self.image_name, path = self.path)
            # self.sense_interpret_bus.write(f'{self.path}/{self.image_name}')
            logging.debug("Photo Taken")
            return f'{self.path}/{self.image_name}'
        except:
            logging.debug("Photo Failed")

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
        try:
            if self.polarity:
                grayscale_values = [grayscale_value - min(grayscale_values) for grayscale_value in grayscale_values] 
            else:
                grayscale_values = [abs(grayscale_value - max(grayscale_values)) for grayscale_value in grayscale_values] 

            left, middle, right = grayscale_values
            logging.debug(f'MODIFIED - Left: {left}, Middle: {middle}, Right: {right}')

            try:
                if left > right:
                    approx_location = (middle - left)/max(left, middle)
                    if approx_location < 0:
                        self.robot_location = approx_location
                        return self.robot_location
                    self.robot_location = approx_location - 1
                    return self.robot_location
                approx_location = (middle-right)/max(middle, right)
                if approx_location < 0:
                    self.robot_location = -1*approx_location
                    return self.robot_location
                self.robot_location = 1-approx_location
                return self.robot_location
            except:
                logging.debug(f'Divide by zero error, continuing')
        except:
            logging.debug(f'Grayscale not initialized, passing')

    def line_location_camera(self, file_name):
        try:
            gray_img = cv2.imread(f'{file_name}.jpg')
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
                return 0

            if M['m00'] != 0:
                # cx = int(M['m10']/M['m00'])
                # cy = int(M['m01']/M['m00'])
                self.robot_location = (int(M['m10']/M['m00']) - img_width)/img_width
                # cv2.circle(gray_img, (cx, cy), 5, (0, 0, 255), -1)
                # cv2.imshow("Gray", gray_img)
                # cv2.waitKey(0)
                #cv2.destroyAllWindows()
                return self.robot_location
        except:
            logging.debug("No image found")

class Control():
        def __init__(self, px, k_p = 30, k_i = 0.0, stop_distance = 15, threshold = 0.1):
            self.px = px
            self.k_p = k_p
            self.k_i = k_i
            self.threshold = threshold
            self.stop_distance = stop_distance
            self.error = 0.0
            self.angle = 0.0
    
        def steer(self, car_position):
            try:
                if abs(car_position) > self.threshold:
                    self.error += car_position
                    self.angle = self.k_p * car_position + self.error * self.k_i
                    logging.debug(f'Steering Angle: {self.angle}')
                    self.px.set_dir_servo_angle(self.angle)
                else:
                    self.angle = 0
                    logging.debug(f'Steering Angle: {self.angle}')
                    self.px.set_dir_servo_angle(self.angle)
            except:
                logging.debug("No steering provided")
        
        def ultrasonic_stop(self, car_distance):
            if car_distance < self.stop_distance:
                px.forward(0)
            else:
                px.forward(40)

if __name__ == "__main__":
    method = 0
    px = Picarx()
    while method != 1 and method != 2:
        method = int(input("Select 1 for grayscale or 2 for camera based line following: "))
    if method == 1:
        camera = False
    elif method == 2:
        camera = True
    
    sense_delay = 0.1
    think_delay = 0.1
    control_delay = 0.1
    print_delay = 0.25
    full_time = 15
    check_time = 0.01

    sense = Sense(px = px, camera = camera)
    think = Interpret(polarity = False)
    control = Control(px = px, threshold = 0.1)
    time.sleep(2)
    
    px.forward(30)
    sense_interpret_bus = ros.Bus(sense.get_grayscale_from_hardware(), "Grayscale hardware")
    interpret_control_bus = ros.Bus(think.line_location_grayscale(sense.get_grayscale_from_hardware()), "Position calculation")
    ultrasonic_bus = ros.Bus(sense.get_ultrasonic_from_hardware(), "Ultrasonic bus")
    terminate_bus = ros.Bus(0, "Termination bus")
       
    read_grayscale = ros.Producer(
        sense.get_grayscale_from_hardware,
        sense_interpret_bus,
        sense_delay,
        terminate_bus,
        "Read grayscale values"
    )

    read_ultrasonic = ros.Producer(
        sense.get_ultrasonic_from_hardware,
        ultrasonic_bus,
        sense_delay,
        terminate_bus,
        "Read Ultrasonic values"
    )

    determine_position = ros.ConsumerProducer(
        think.line_location_grayscale,
        sense_interpret_bus,
        interpret_control_bus,
        think_delay,
        terminate_bus,
        "Calculate position"
    )

    determine_stop = ros.ConsumerProducer(
        control.ultrasonic_stop,
        ultrasonic_bus,
        terminate_bus,
        think_delay,
        terminate_bus,
        "Calculate position"
    )

    move_wheels = ros.Consumer(
        control.steer,
        interpret_control_bus,
        control_delay,
        terminate_bus,
        "Steer"
    )

    print_buses = ros.Printer(
        (sense_interpret_bus, interpret_control_bus, terminate_bus),
        print_delay,
        terminate_bus,
        "Print raw data",
        "Data bus readings are: "
    )

    terminate_timer = ros.Timer(
        terminate_bus,
        full_time,
        check_time,
        terminate_bus,
        "Termination Timer"
    )

    producer_consumer_list = [read_grayscale,
                              determine_position,
                              move_wheels,
                                #print_buses,
                                terminate_timer,
                              determine_stop,
                              read_ultrasonic 
                              ]
    
    ros.runConcurrently(producer_consumer_list)