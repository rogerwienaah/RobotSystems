from picarx_improved import Picarx
from time import sleep
import readchar

manual = '''
Press keys on keyboard to control the PiCar-X!
    m: random basic movements
    p: parallel parking
    k: K turning
'''


def show_info():
    print("\033[H\033[J",end='')  # clear terminal windows
    print(manual)

# some random movements 
def basic_maneuvering():
    px.set_dir_servo_angle(45)
    sleep(1)

    px.set_dir_servo_angle(-45)
    sleep(1)

    px.stop()





if __name__ == "__main__":
    px = Picarx()
    show_info()
    basic_maneuvering()
