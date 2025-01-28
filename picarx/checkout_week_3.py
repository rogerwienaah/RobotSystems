
from picarx_improved import Picarx
from time import sleep
import logging

logging_format = '%(asctime)s: %(message)s'
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt='%H:%M:%S')
logging.getLogger().setLevel(logging.DEBUG)




class Sensor:
    def __init__(self):
        self.px = Picarx()
        self.current_state = None
        self.px_power = 10
        self.offset = 20
        self.last_state = "stop"


    # get grayscale reading
    def grayscale_reading(self):
        self.gm_val_list = self.px.get_grayscale_data()
        self.gm_state = self.get_status(self.gm_val_list)

        logging.debug("gm_val_list: %s"%(self.gm_val_list))
        logging.debug("gm_state: %s"%(self.gm_state))


    # get status of the robot with respect to line
    def get_status(self, val_list):
        self._state = self.px.get_line_status(val_list)  # [bool, bool, bool], 0 means line, 1 means background
        
        logging.debug("gm_status: %s"%(self._state))

        # get location of robot with respect to line / grayscale sensor on line
        if self._state == [0, 0, 0]:
            return 'stop'
        elif self._state[1] == 1:
            return 'forward'
        elif self._state[0] == 1:
            return 'right'
        elif self._state[2] == 1:
            return 'left'

    # handling the out of line situation
    def outHandle(self):
        if self.last_state == 'left':
            self.px.set_dir_servo_angle(-30)
            self.px.backward(10)
        elif self.last_state == 'right':
            self.px.set_dir_servo_angle(30)
            self.px.backward(10)
        while True:
            gm_val_list = self.px.get_grayscale_data()
            gm_state = self.get_status(gm_val_list)
            logging.debug("outHandle gm_val_list: %s, %s"%(gm_val_list, gm_state))
            self.currentSta = gm_state
            if self.currentSta != self.last_state:
                break
            sleep(0.001)


    def control_loop(self):
        try:
            while True:
                self.grayscale_reading()

                if self.gm_state != "stop":
                    self.last_state = self.gm_state

                if self.gm_state == 'forward':
                    self.px.set_dir_servo_angle(0)
                    self.px.forward(self.px_power) 
                elif self.gm_state == 'left':
                    self.px.set_dir_servo_angle(self.offset)
                    self.px.forward(self.px_power) 
                elif self.gm_state == 'right':
                    self.px.set_dir_servo_angle(-self.offset)
                    self.px.forward(self.px_power) 
                else:
                    self.outHandle()

        finally:
            self.px.stop()
            logging.info("stop and exit")
            sleep(0.1)



if __name__=='__main__':
    line_follower = Sensor()
    line_follower.control_loop()
