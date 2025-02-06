from picarx_improved import Picarx
import time
from line_following import Sense, Interpret, Control



def sensor_producer(busIns, time_delay):
    while True:
        sensor_data = Sense.get_grayscale_data()
        busIns.write(sensor_data)
        time.sleep(time_delay)



def interprete_cons_prod(busIns, time_delay):
    sensor_data = busIns.read()
    line_status = Interpret.get_line_status_grayscale(sensor_data)
    pass

def control_cons(busIns, time_delay):
    
    
    pass



class Bus():
    def __init__(self):
        self.message = None
        
        
    def write(self, message):
        self.message = message

    def read(self):
        return self.message
