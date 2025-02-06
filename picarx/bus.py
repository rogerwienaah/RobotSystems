from picarx_improved import Picarx
import time
from line_following import Sense, Interpret, Control
from concurrent.futures import ThreadPoolExecutor
from threading import Event



def sensor_producer(busIns, time_delay):
    while True:
        sensor_data = Sense.get_grayscale_data()
        busIns.write(sensor_data)
        time.sleep(time_delay)



def interprete_cons_prod(busIns, time_delay):
    while True:
        sensor_data = busIns.read()
        line_status = Interpret.get_line_status_grayscale(sensor_data)
        busIns.write(line_status)
        time.sleep(time_delay)
    

def control_cons(busIns, time_delay):
    while True:
        line_status = busIns.read()
        control = Control.steer(line_status)
        time.sleep(time_delay)



class Bus():
    # initialize class
    def __init__(self):
        self.message = None
        
    
    # write to bus
    def write(self, message):
        self.message = message

    # read from bus
    def read(self):
        return self.message



if __name__ == "__main__":

    with ThreadPoolExecutor(max_workers=2) as executor:
        eSensor = executor.submit(sensor_producer, Bus.write(), time_delay)
        eInterpreter = executor.submit(interprete_cons_prod, Bus.write(), Bus.read(),interpreter_delay)