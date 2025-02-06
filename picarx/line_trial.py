from picarx_improved import Picarx
from time import sleep

class Sensor():
    def __init__(self, px):
        self.px = px
    
    def get_grayscale_data(self):
        return self.px.get_grayscale_data()

class Interpret():
    def __init__(self, px):
        self.px = px
    
    def get_status(self, val_list):
        _state = self.px.get_line_status(val_list)  # [bool, bool, bool], 0 means line, 1 means background
        if _state == [0, 0, 0]:
            return 'stop'
        elif _state[1] == 1:
            return 'forward'
        elif _state[0] == 1:
            return 'right'
        elif _state[2] == 1:
            return 'left'
        else:
            return 'stop'  # Default to stop if no condition is met

class Control():
    def __init__(self, picarx, sensor_reader, line_interpreter, px_power=80, offset=20):
        self.px = picarx
        self.sensor_reader = sensor_reader
        self.line_interpreter = line_interpreter
        self.px_power = px_power
        self.offset = offset
        self.last_state = "stop"
    
    def out_handle(self):
        if self.last_state == 'left':
            self.px.set_dir_servo_angle(-30)
            self.px.backward(self.px_power)
        elif self.last_state == 'right':
            self.px.set_dir_servo_angle(30)
            self.px.backward(self.px_power)
        
        max_iterations = 100  # Prevent infinite loop
        for _ in range(max_iterations):
            gm_val_list = self.sensor_reader.get_grayscale_data()
            gm_state = self.line_interpreter.get_status(gm_val_list)
            print("outHandle gm_val_list: %s, %s" % (gm_val_list, gm_state))
            if gm_state != self.last_state:
                break
            sleep(0.01)  # Increased sleep duration
    
    def run(self):
        try:
            while True:
                gm_val_list = self.sensor_reader.get_grayscale_data()
                gm_state = self.line_interpreter.get_status(gm_val_list)
                print("gm_val_list: %s, %s" % (gm_val_list, gm_state))
                
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
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            self.px.stop()
            print("stop and exit")
            sleep(0.1)

if __name__ == '__main__':
    px = Picarx()
    sensor_reader = Sensor(px)
    line_interpreter = Interpret(px)
    controller = Control(px, sensor_reader, line_interpreter)
    controller.run()