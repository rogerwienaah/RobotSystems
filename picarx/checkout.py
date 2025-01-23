from picarx_improved import Picarx
from time import sleep
import readchar

manual = '''
Press keys on keyboard to control PiCar-X!
    a: Move forward / backwards
    b: parallel parking
    c: K turning
'''

def show_info():
    print("\033[H\033[J",end='')  # clear terminal windows
    print(manual)


if __name__ == "__main__":
    try:
        px = Picarx()
        show_info()

        while True:
            key = readchar.readkey()
            key = key.lower()
            if key in('abc'): 

                # move forward & backward
                if 'a' == key:
                    px.set_dir_servo_angle(0)
                    px.forward(80)
                    sleep(3)

                    px.set_dir_servo_angle(30)
                    px.forward(80)
                    sleep(1)

                    px.set_dir_servo_angle(-30)
                    px.backward(80)

                    px.set_dir_servo_angle(0)
                    px.backward(80)
                    sleep(1)

                # parallel parking
                elif 'b' == key:
                    px.set_dir_servo_angle(0)
                    px.forward(80)
                    sleep(2)

                    px.set_dir_servo_angle(30)
                    px.forward(80)
                    sleep(4)

                    px.set_dir_servo_angle(0)
                    px.forward(80)
                    sleep(2)

                    px.set_dir_servo_angle(30)
                    px.backward(80)
                    sleep(3)

                    # px.set_dir_servo_angle(30)
                    # px.backward(80)
                    # sleep(3)

                    px.set_dir_servo_angle(0)
                    px.backward(80)
                    sleep(2)

                    px.backward(0)
                
                #3-point turn
                elif 'c' == key:
                    px.set_dir_servo_angle(30)
                    px.forward(80)
                    sleep(4)

                    px.set_dir_servo_angle(30)
                    px.backward(80)
                    sleep(4)

                    px.set_dir_servo_angle(-30)
                    px.forward(80)
                    sleep(4)

                    px.forward(0)


          
            elif key == 'q':
                print("\n Quit")
                break

    finally: 
        px.set_dir_servo_angle(0)  
        px.stop()
        sleep(.2)


