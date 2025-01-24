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
    px.set_dir_servo_angle(0)
    px.forward(80)
    sleep(1)

    px.set_dir_servo_angle(30)
    px.forward(80)
    sleep(1)

    px.set_dir_servo_angle(-30)
    px.backward(80)
    sleep(1)

    px.set_dir_servo_angle(0)
    px.backward(80)
    sleep(1)

    px.stop()

# parallel parking
def parallel_parking():
    px.set_dir_servo_angle(0)
    px.forward(80)
    sleep(0.3)

    px.set_dir_servo_angle(45)
    px.forward(80)
    sleep(1)

    px.set_dir_servo_angle(-45)
    px.forward(80)
    sleep(1)

    px.set_dir_servo_angle(0)
    px.backward(80)
    sleep(0.2)

    px.set_dir_servo_angle(45)
    px.backward(80)
    sleep(1)

    px.set_dir_servo_angle(-45)
    px.backward(80)
    sleep(1)

    px.set_dir_servo_angle(0)
    px.backward(80)
    sleep(0.4)

    px.stop()

# 3 point turning
def k_turning():
    px.set_dir_servo_angle(0)
    px.forward(80)
    sleep(0.4)

    px.set_dir_servo_angle(45)
    px.forward(80)
    sleep(1)

    px.set_dir_servo_angle(-45)
    px.backward(80)
    sleep(1)

    px.set_dir_servo_angle(45)
    px.forward(80)
    sleep(1)

    px.set_dir_servo_angle(0)
    px.stop()



if __name__ == "__main__":
    px = Picarx()
    show_info()

    while True:
        key = readchar.readkey()
        key = key.lower()
        px.set_dir_servo_angle(0)

        if key in('mpk'): 

            # move forward / backwards with some steering - basic maneuvering
            if 'm' == key:
                basic_maneuvering()

            # parallel parking
            elif 'p' == key:
                parallel_parking()
            
            #3-point turn
            elif 'k' == key:
                k_turning()
    
        elif key == 'q':
            print("\n Quit")
            break
 



