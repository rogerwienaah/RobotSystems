from robot_hat import ADC
from robot_hat import Grayscale_Module


class SENSOR():

    def __init__(self, Picar):
        self.px = Picar

    def sensor_read(self):
        sensor_values = self.px.get_grayscale_data()
        print("Sensor values:", sensor_values)
        return sensor_values
    

class INTERP():

    def __init__(self, px,sensitivity = 400, polarity = 1, reference = [500,500,500]):
        #If line is darker than background, use default polarity
        #If line is lighter, set polarity to 0
        self.px = px

        
        self.sensitivity = sensitivity
        self.polarity = polarity
        self.reference = self.px.set_grayscale_reference(reference)
        self.ref = reference[0]

    def get_position(self,sensor_val_list):
        sensitivity = self.sensitivity
        sv = sensor_val_list
        #Reverse everything if polarity switched
        if self.polarity == 0:
            sv = sv*-1
        #Compare left and middle:
        ref = self.ref
        Left = sv[0]
        Middle = sv[1]
        Right = sv[2]
        dif1 = Middle - Left
        dif2 = Right - Middle
        position = None
        if abs(dif1) > sensitivity and abs(dif2) > sensitivity and dif1 < 0:
            print("Car is centered")
            position = 0
        elif abs(dif1) > sensitivity and abs(dif2) < sensitivity:
            if dif1 > 0:
                print("Car is to the right")
                position = 1*(abs(dif1)/sensitivity)
            elif dif1 < 0:
                print("Car at edge case to left")
                position = -0.5*(abs(dif1)/sensitivity) 
        elif abs(dif1) < sensitivity and abs(dif2) > sensitivity:
            if dif2 > 0:
                print("car at edge case to the right")
                position = 0.5*(abs(dif2)/sensitivity)
            elif dif2 < 0:
                print("car is to the left")
                position = -1*(abs(dif2)/sensitivity)
        else:
            position = 0
        return position 
    

class CONTROL():

    def __init__(self,Picar, scale_factor = 20):
        #Negative to correct the offset
        self.scale_factor = -1*scale_factor
        self.px = Picar

    def correct_car(self,offset):
        servo_angle = offset*self.scale_factor
        print("Servo angle:", servo_angle)
        self.px.set_dir_servo_angle(servo_angle)






            

      

 






