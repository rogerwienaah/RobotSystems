from picarx_improved import Picarx
from time import sleep
import readchar
import logging

# logging configuration
logging_format = '%(asctime)s: %(message)s'
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt='%H:%M:%S')
logging.getLogger().setLevel(logging.DEBUG)



class Maneuvering:

    def __init__(self):
        self.px = Picarx()
        self.manual = '''
        Press keys on keyboard to control the PiCar-X!
            m: random basic movements
            p: parallel parking
            k: K turnings
        '''

    def show_info(self):
        print("\033[H\033[J",end='')  # clear terminal windows
        print(self.manual)

    # some random movements 
    def basic_maneuvering(self):
        self.px.set_dir_servo_angle(0)
        self.px.forward(80)
        sleep(1)

        self.px.set_dir_servo_angle(30)
        self.px.forward(80)
        sleep(1)

        self.px.set_dir_servo_angle(-30)
        self.px.backward(80)
        sleep(1)

        self.px.set_dir_servo_angle(0)
        self.px.backward(80)
        sleep(1)

        self.px.stop()

    # parallel parking
    def parallel_parking(self):
        logging.info("Parallel parking started")
        self.px.set_dir_servo_angle(0)
        self.px.forward(80)
        sleep(0.3)

        self.px.set_dir_servo_angle(45)
        self.px.forward(80)
        sleep(1)

        self.px.set_dir_servo_angle(-45)
        self.px.forward(80)
        sleep(1)

        self.px.set_dir_servo_angle(45)
        self.px.backward(80)
        sleep(1)

        self.px.set_dir_servo_angle(-45)
        self.px.backward(80)
        sleep(1)

        self.px.set_dir_servo_angle(0)
        self.px.backward(80)
        sleep(1)

        self.px.stop()
        logging.info("Parallel parking completed")

    # 3 point turning
    def k_turning(self):
        logging.info("K-turning started")
        self.px.set_dir_servo_angle(0)
        self.px.forward(80)
        sleep(0.4)

        self.px.set_dir_servo_angle(45)
        self.px.forward(80)
        sleep(1.5)

        self.px.set_dir_servo_angle(-45)
        self.px.backward(80)
        sleep(1.5)

        self.px.set_dir_servo_angle(45)
        self.px.forward(80)
        sleep(1.5)

        self.px.set_dir_servo_angle(0)
        self.px.stop()
        logging.info("K-turning completed")

    def control_loop(self):
        self.show_info()

        # user input
        while True:
            key = readchar.readkey()
            key = key.lower()
            self.px.set_dir_servo_angle(0)

            if key in('mpk'): 

                if 'm' == key:
                    self.basic_maneuvering()

                elif 'p' == key:
                    self.parallel_parking()
                
                elif 'k' == key:
                    self.k_turning()
        
            elif key == 'q':
                print("\n Quit")
                break

if __name__ == "__main__":
    maneuver = Maneuvering()
    maneuver.control_loop()
 



