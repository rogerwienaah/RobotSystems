# from robot_hat import Pin, ADC, PWM, Servo, fileDB
# from robot_hat import Grayscale_Module, Ultrasonic, utils

import os
import logging
import atexit
import math

try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic, utils
except ImportError:
    import sys
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic, utils

import time

# logging configuration
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO,
datefmt="%H:%M:%S")
# logging.getLogger().setLevel(logging.DEBUG)
# logging.debug(message)


def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: trig, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P13', 'P12'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # reset robot_hat
        utils.reset_mcu()
        time.sleep(0.2)

        # --------- config_flie ---------
        on_the_robot = False
        if on_the_robot:
            self.config_flie = fileDB(config, 777, os.getlogin())
        else:
            self.config_flie = fileDB(config)

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        trig, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(trig), Pin(echo, mode=Pin.IN, pull=Pin.PULL_DOWN))
        
    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # print(f"direction: {direction}, speed: {speed}")
        if speed != 0:
            # speed = int(speed /2 ) + 50 # removed speed scaling
            speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def backward(self, speed):
        logging.debug("Moving backward")
        current_angle = self.dir_current_angle
        if current_angle != 0:
            ackermann_scale = self.ackermann_steering_angle(current_angle) # using ackermann steering angle
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            # power_scale = (100 - abs_current_angle) / 100.0 


            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed * ackermann_scale[1])
                self.set_motor_speed(2, speed * ackermann_scale[0])
            else:
                self.set_motor_speed(1, -1*speed * ackermann_scale[0])
                self.set_motor_speed(2, speed * ackermann_scale[1])
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed) 

        logging.debug("Done Moving forward")  

    def forward(self, speed):
        logging.debug("Moving forward")
        current_angle = self.dir_current_angle
        if current_angle != 0:
            ackermann_scale = self.ackermann_steering_angle(current_angle)
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            # power_scale = (100 - abs_current_angle) / 100.0
            

            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, 1*speed * ackermann_scale[1])
                self.set_motor_speed(2, -speed * ackermann_scale[0]) 
            else:
                self.set_motor_speed(1, speed * ackermann_scale[0])
                self.set_motor_speed(2, -1*speed * ackermann_scale[1])
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed) 

        logging.debug("Done Moving forward")                

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        logging.debug("PiCar Stopped")
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def reset(self):
        self.stop()
        self.set_dir_servo_angle(0)
        self.set_cam_tilt_angle(0)
        self.set_cam_pan_angle(0)
    
    

    def ackermann_steering_angle(self, steering_angle):
        # values for wheelbase and trackwidth from picar schematics
        logging.debug("Calculating ackermann steering angle")

        wheelbase = 93.88
        track_width = (142.65 - 27.42)
        
        x = wheelbase / math.tan(abs(steering_angle) * (math.pi / 180))

        

        scale_inner = (x - (track_width / 2 )) / x
        scale_outer = (x + ( track_width / 2)) / x
        
        # x = wheelbase * math.tan(abs(steering_angle) * (math.pi / 180))
        # RICR = x + (track_width /2)

        # front_wheel_right = math.sqrt((wheelbase ** 2) + (x ** 2))
        # front_wheel_left = math.sqrt((wheelbase ** 2) + (track_width + x) ** 2)

        logging.debug("Done calculating ackermann steering angle")

        return [scale_inner, scale_outer]





        
    

if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    time.sleep(1)
    atexit.register(px.stop)
    
