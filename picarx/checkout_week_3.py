from picarx_improved import Picarx
from time import sleep
import logging

# Logging configuration
logging_format = '%(asctime)s: %(message)s'
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt='%H:%M:%S')
logging.getLogger().setLevel(logging.DEBUG)


class Sensor:
    """Handles sensor readings from the robot"""
    def __init__(self, picarx):
        self.px = picarx

    def get_grayscale_reading(self):
        """Returns grayscale sensor readings"""
        gm_val_list = self.px.get_grayscale_data()
        logging.debug("gm_val_list: %s" % gm_val_list)
        return gm_val_list


class Interpreter:
    """Interprets sensor data to determine movement direction"""
    def __init__(self, picarx):
        self.px = picarx

    def get_status(self, val_list):
        """Determines the status of the robot with respect to the line"""
        state = self.px.get_line_status(val_list)  # [bool, bool, bool]
        logging.debug("gm_status: %s" % state)

        if state == [0, 0, 0]:
            return 'stop'
        elif state[1] == 1:
            return 'forward'
        elif state[0] == 1:
            return 'right'
        elif state[2] == 1:
            return 'left'


class Controller:
    """Controls the robot based on sensor input"""
    def __init__(self):
        self.px = Picarx()
        self.sensor = Sensor(self.px)
        self.interpreter = Interpreter(self.px)
        
        self.px_power = 10
        self.offset = 20
        self.last_state = "stop"

    def out_handle(self):
        """Handles the situation when the robot goes off the line"""
        if self.last_state == 'left':
            self.px.set_dir_servo_angle(-30)
            self.px.backward(10)
        elif self.last_state == 'right':
            self.px.set_dir_servo_angle(30)
            self.px.backward(10)

        while True:
            gm_val_list = self.sensor.get_grayscale_reading()
            gm_state = self.interpreter.get_status(gm_val_list)
            logging.debug("outHandle gm_val_list: %s, %s" % (gm_val_list, gm_state))

            if gm_state != self.last_state:
                break
            sleep(0.001)

    def control_loop(self):
        """Main loop that continuously controls the robot"""
        try:
            while True:
                gm_val_list = self.sensor.get_grayscale_reading()
                gm_state = self.interpreter.get_status(gm_val_list)

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
            logging.info("stop and exit")
            sleep(0.1)


if __name__ == '__main__':
    controller = Controller()
    controller.control_loop()