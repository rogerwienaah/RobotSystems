from picarx_improved import Picarx
from vilib import Vilib
import time
import logging
import cv2

import rossros as rr


# logging configuration
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level = logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)



class Sense():
    def __init__(self, px, camera = False):
        self.px = px
        
        if camera:
            Vilib.camera_start()
            time.sleep(0.5)
            self.path = "picarx"
            self.image = "image"
            self.px.set_cam_tilt_angle(-30)
            #Vilib.display()
    

    def get_grayscale_data(self):
        try:
            return self.px.get_grayscale_data()
        except:
            logging.debug("Failed to get Grayscale Data")


    def get_camera_image(self):
        try:
            Vilib.capture_image(self.image, self.path)
        except:
            logging.debug("Failed to Capture Image")


    def get_ultrasonic_data(self):
        try:
            return self.px.get_distance()
        except:
            logging.debug("Failed to get Ultrasonic Data")





class Interpret():
    def __init__(self, px, polarity = False):
        self.px = px
        self.polarity = polarity #False - dark line, True - White line

        self.robot_location = 0
        self.thresh = 75
        self.colour = 255

    

    def grayscale_line_status(self, val_list):
        try: 
            _state = self.px.get_line_status(val_list)  # [bool, bool, bool], 0 means line, 1 means background
            if _state == [0, 0, 0]:
                return 'stop'
            elif _state[1] == 1:
                return 'forward'
            elif _state[0] == 1:
                return 'right'
            elif _state[2] == 1:
                return 'left'
        except:
            logging.debug("Failed to get line status")


    def camera_line_status(self, fname):
        try:
            bnw_image = cv2.imread(f'{fname}.jpg')
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
                return 0 

            if M['m00'] != 0:
                self.robot_location = (int(M['m10']/M['m00']) - img_width)/img_width
                return self.robot_location
            
        except:
            logging.debug("Failed to get robot location")




class Control():
        def __init__(self, px, sensor_reader, line_interpreter, stop_distance = 15, k_p = 30, k_i = 0.0, k_d = 5.0, threshold = 0.1):
            self.px = px
            
            # PI / PID control parameters
            self.k_p = k_p # Proportional gain
            self.k_i = k_i  # Integral gain
            self.k_d = k_d  # Derivative gain

            self.error = 0.0
            self.angle = 0.0
            self.prev_error = 0.0 # Previous error for derivative calculation

            self.threshold = threshold
            self.last_time = time.time() # Store last timestamp for derivative calculation

            # Ultrasonic sensor stop distance
            self.stop_distance = stop_distance

            # Gray scale line following parameters
            self.sensor_reader = sensor_reader
            self.line_interpreter = line_interpreter
            self.px_power = 80
            self.offset = 20
            self.last_state = "stop"

           
            
        # PI control to adjust the steering of the picar
        def steer_PI(self, picar_position):
            try:
                if abs(picar_position) > self.threshold:
                    self.error += picar_position
                    self.angle = self.k_p * picar_position + self.error * self.k_i
                    logging.debug(f'Steering Angle: {self.angle}')
                    self.px.set_dir_servo_angle(self.angle)
                else:
                    self.angle = 0
                    logging.debug(f'Steering Angle: {self.angle}')
                    self.px.set_dir_servo_angle(self.angle)
            except:
                logging.debug("No steering provided")
        

        def ultrasonic_stop(self, picar_distance):
            if picar_distance < self.stop_distance:
                return 1
            return 0
        

        # PID control to adjust the steering of the picar
        def steer_PID(self, picar_position):
            try:
                current_time = time.time()
                dt = current_time - self.last_time  # Time difference for derivative term
                if dt == 0:  # Prevent division by zero
                    dt = 0.001


                if abs(picar_position) > self.threshold:
                    proportional = self.k_p * picar_position 
                    self.error += picar_position * dt 
                    integral = self.k_i * self.error
                    derivative = self.k_d * ((picar_position - self.prev_error) / dt)

                    self.angle = proportional + integral + derivative # total steering angle

                    logging.debug(f'P: {proportional}, I: {integral}, D: {derivative}, Angle: {self.angle}')

                    self.px.set_dir_servo_angle(self.angle)
                    self.prev_error = picar_position  # Store error for next derivative calculation
                    self.last_time = current_time

                else:
                    self.angle = 0
                    self.px.set_dir_servo_angle(self.angle)
            except:
                logging.debug("No steering provided")


        # Control for grayscale line following - handles when car is out of the line
        def grayscale_out_handle(self):
            if self.last_state == 'left':
                self.px.set_dir_servo_angle(-30)
                self.px.backward(10)
            elif self.last_state == 'right':
                self.px.set_dir_servo_angle(30)
                self.px.backward(10)
            while True:
                gm_val_list = self.sensor_reader.get_grayscale_data()
                gm_state = self.line_interpreter.get_status(gm_val_list)
                print("outHandle gm_val_list: %s, %s" % (gm_val_list, gm_state))
                

                if gm_state != self.last_state:
                    break
            time.sleep(0.001) # -- tweak this value
        
        # Control for grayscale line following - handles when car is in the line
        def steer_with_grayscale(self):
            try:
                while True:
                    gm_val_list = self.sensor_reader.get_grayscale_data()
                    gm_state = self.line_interpreter.get_status(gm_val_list)
                    print("gm_val_list: %s, %s" % (gm_val_list, gm_state))
                    
                    if gm_state != "stop":
                        self.last_state = gm_state
                    
                    if gm_state == 'forward':
                        self.px.set_dir_servo_angle(0)
                        self.px.forward(self.px_power)
                    elif gm_state == 'left':
                        self.px.set_dir_servo_angle(self.offset)
                        self.px.forward(self.px_power)
                    elif gm_state == 'right':
                        self.px.set_dir_servo_angle(-self.offset)
                        self.px.forward(self.px_power)
                    else:
                        self.out_handle()
            finally:
                self.px.stop()
                print("stop and exit")
                time.sleep(0.1)





if __name__ == "__main__":

    px = Picarx()
    choice = 0

    while choice != 1 and choice != 2:
        choice = int(input("Heyya, Select 1 for grayscale or 2 for camera based line following: "))
    
    if choice == 1:
        camera = False
    elif choice == 2:
        camera = True


    sense = Sense(px, camera = camera)
    interpret = Interpret(px, polarity = False)
    control = Control(px, sense, interpret, threshold = 0.01)

    sense_delay = 0.1
    control_delay = 0.1
    think_delay = 0.1
    interpret_delay = 0.1
    print_delay = 0.25
    full_time = 15
    check_time = 0.01
    
    time.sleep(2)

    px.forward(30)
    

    # Grayscale Ross ros 
    sense_bus = rr.Bus(sense.get_grayscale_data(), "Grayscale Data Bus")
    interpret_control_bus = rr.Bus(interpret.grayscale_line_status(sense.get_grayscale_data()), "Interpret / Control Bus")
    bus_terminate = rr.Bus(0, "Termination Bus")
    ultrasonic_bus = rr.Bus(sense.get_ultrasonic_data(), "Ultrasonic bus")



    read_grayscale_data = rr.Producer(
        sense.get_grayscale_data, 
        sense_bus, 
        sense_delay, 
        bus_terminate, 
        "Read Sensor Data")
    
    read_ultrasonic_data = rr.Producer(
        sense.get_ultrasonic_data,
        ultrasonic_bus,
        sense_delay,
        bus_terminate,
        "Read Ultrasonic Data"
    )

    
    find_picar_position = rr.ConsumerProducer(
        interpret.grayscale_line_status,
        sense_bus,
        interpret_control_bus,
        think_delay,
        bus_terminate,
        "Find picar position"
    )

    # --check this again - bus_terminate
    find_stop = rr.ConsumerProducer(
        control.ultrasonic_stop,
        ultrasonic_bus,
        bus_terminate,
        think_delay,
        bus_terminate,
        "Find picar position for stopping"
    )

    move_picar = rr.Consumer(
        control.steer_with_grayscale,
        interpret_control_bus,
        control_delay,
        bus_terminate,
        "Move picar"
    )

    print_buses = rr.Printer(
        (sense_bus, interpret_control_bus, bus_terminate),
        print_delay,
        bus_terminate,
        "Print raw data",
        "Data bus readings are: "
    )

    terminate_timer = rr.Timer(
        bus_terminate,
        full_time,
        check_time,
        bus_terminate,
        "Termination Timer"
    )


    producer_consumer_list = [read_grayscale_data,
                              find_picar_position,
                              move_picar,
                              print_buses,
                              terminate_timer,
                              find_stop,
                              read_ultrasonic_data
                              ]
    
    rr.runConcurrently(producer_consumer_list)

