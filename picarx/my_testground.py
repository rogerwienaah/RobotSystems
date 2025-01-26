from picarx_improved import Picarx
from time import sleep
import readchar

px = Picarx()

def basic_maneuvering():
    px.set_dir_servo_angle(0)
    px.forward(50)
    sleep(1)


    px.set_dir_servo_angle(45)
    px.forward(50)
    sleep(2)

    px.set_dir_servo_angle(-45)
    px.forward(50)
    sleep(2)

    px.set_dir_servo_angle(0)
    px.forward(50)
    sleep(1)

    px.stop()





if __name__ == "__main__":
    basic_maneuvering()
