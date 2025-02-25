
import logging
from logdecorator import log_on_start, log_on_end, log_on_error
from _picarx_improve import Picarx
import atexit
import time
import rossros as rr
from bus_structure import Bus_Structure

import concurrent.futures

logging.getLogger().setLevel(logging.INFO)

px = Picarx()



class Sensor():
    """
    Sensor class for line following
    """
    
    def __init__(self):
        pass
        
        
    def read(self):
        logging.debug("Got data")    
        return px.grayscale.read()
    

    def read_producer(self,Reading_Bus_class,delay):
        #producer function that writes grayscale data
        while True:
            try:
                Reading_Bus_class.write(px.grayscale.read())
                logging.info("Got data") 
                time.sleep(delay)
            except:
                logging.debug("Not initialized --- skipping")
    
class Interpreter():

    def __init__(self,sensitivity=200,polarity=1):
        self.sensitivity = sensitivity
        self.polarity = polarity

    def process(self,Reading):
        #Identify if there is a sharp or not sharp change

        #Use edge location and sign to determine left or right and centered or not centered

        #Make robust to different lighting conditions, option to target darker or lighter
        #Bruh

        #Might need to calibrate this first, we will see

        #Start by finding edges, directionality may need to be changed later


        #Positive values should be to the left

        #Low numbers mean black, so negative readings mean the thing is to the left

        Left = Reading[0]-50
        Middle = Reading[1]-180
        Right = Reading[2]

        edgeleft = Left-Middle #Difference between left reading and middle
        edgeright = Right-Middle #Difference between right and middle
        edgehigh = Left-Right

        threshold = self.sensitivity #Set threshold based on sensitivity value
        
        large_const = 1/2000 #Constant used to scale large edge
        med_const = 1/2000 #Constant use for off by a medium amount
        close_const = 1/1000 #Constant used for very close

        #TODO: Before moving on we need to adjust using the polarity setting 

        
        if abs(edgehigh)>threshold:
            #If the extremes are over, we know that we are far off
            Distance = edgehigh*large_const*-1 #built in lef tor right logic
                
        else:
           
            Distance = edgeleft*med_const*-1
            #Add logic for left or right
       
            
        return Distance

    def process_consumer_producer(self,Reading_Bus_class,Distance_Bus_Class,delay):
        #Consumer-producer function for reading grayscale sensor and publishing distance

        while True:
            try:
                #get reading from bus
                Reading = Reading_Bus_class.read()

                Left = Reading[0]-50
                Middle = Reading[1]-180
                Right = Reading[2]

                edgeleft = Left-Middle #Difference between left reading and middle
                edgeright = Right-Middle #Difference between right and middle
                edgehigh = Left-Right

                threshold = self.sensitivity #Set threshold based on sensitivity value
                
                large_const = 1/2000 #Constant used to scale large edge
                med_const = 1/2000 #Constant use for off by a medium amount
                close_const = 1/1000 #Constant used for very close
                if abs(edgehigh)>threshold:
                #If the extremes are over, we know that we are far off
                    Distance = edgehigh*large_const*-1 #built in lef tor right logic
                    
                else:
                
                    Distance = edgeleft*med_const*-1
                    #Add logic for left or right

                Distance_Bus_Class.write(Distance)
                logging.info("Got Distance") 
                logging.info(Distance)
                time.sleep(delay)
            except:
                logging.debug("Not initialized --- skipping")
            

class Control():

    def __init__(self,kp=60,target = 0,sat=25):
        self.kp = kp
        self.sat = sat
        self.target = target
    
    def proportional_control(self,distance):
        #Lets implement a simple proportional controller for now
        err = self.target-distance
        angle_set = err*self.kp
        #Saturate, though technically this is done for us
        if angle_set>25:
            angle_set=25
        if angle_set <-25:
            angle_set = -25

        px.set_dir_servo_angle(angle_set)
        #px.forward(40)
        

    def proportional_control_consumer_producer(self,Distance_Bus_Class,Angle_Bus_Class,delay):
        #Consumer-producer function for reading Distance sensor and publishing Angle based on P-control
        while True:
            try:
                distance = Distance_Bus_Class.read()

                #Lets implement a simple proportional controller for now
                err = self.target-distance
                angle_set = err*self.kp
                #Saturate, though technically this is done for us
                if angle_set>25:
                    angle_set=25
                if angle_set <-25:
                    angle_set = -25
                logging.info("Got Angle") 
                logging.info(angle_set)
                Angle_Bus_Class.write(angle_set)
                px.set_dir_servo_angle(angle_set)
                #px.forward(30)
                time.sleep(delay)
            except:
                logging.debug("Not initialized --- skipping")


class Ultrasonic_Sensor():
    def __init__(self):
        pass
    def read(self):
        logging.info("Got Ultrasonic Reading")    
        reading = px.get_distance()
        logging.info(reading)    
        return reading
    

class Ultrasonic_Interpreter():
    def __init__(self):
        self.count = 0
        pass

    def process(self,reading):
        logging.info("Processing Ultrasonic Reading") 
        return reading
    
class Ultrasonic_Controller():
    def __init__(self):
        pass

    def control(self,data):
        logging.info("Controlling Ultrasonic Reading") 
        try:
            if data < 15 and data>0:
                #Stop robot
                self.count = self.count+1
                if self.count >5:
                    logging.info("Stopping") 
                    px.forward(0)
                else:
                    px.forward(40)
            else:
                #Make robot go
                px.forward(40)
                self.count = 0
        except:
            pass
        

# if __name__=='__main__':

#     atexit.register(px.stop)


#     sn = Sensor()
#     int = Interpreter()
#     con = Control()
#     time.sleep(1)
#     while True:
        
#         #Read Grayscale module, comment out list for sim
#         Reading = sn.read()
#         #Reading = [2571,3085,3599]

#         logging.debug("Got Reading:")
#         logging.debug(Reading) 
#         Dist = int.process(Reading)
#         logging.debug("Got Dist:")
#         logging.debug(Dist) 

#         Angle = con.proportional_control(Dist)
#         logging.debug("Got Angle:")
#         logging.debug(Angle)
        
#         #TODO: Put these back in when following a line
#         px.set_dir_servo_angle(Angle)
#         px.forward(40)

#         time.sleep(.05)

if __name__=='__main__':

    atexit.register(px.stop)
    sn = Sensor()
    int = Interpreter()
    con = Control()
    usn = Ultrasonic_Sensor()
    uint = Ultrasonic_Interpreter()
    ucon = Ultrasonic_Controller()

    bread = rr.Bus(sn.read(), "Grayscale reading Bus")
    bprocess = rr.Bus(int.process, "Data processing Bus")
    bcontrol = rr.Bus(con.proportional_control, "Control bus")
    buread = rr.Bus(usn.read(), "Ultrasonic reading Bus")
    buprocess = rr.Bus(uint.process, "US Data processing Bus")
    bucontrol = rr.Bus(ucon.control, "US Control bus")
    bTerminate = rr.Bus(0, "Termination Bus")


    # Wrap the square wave signal generator into a producer
read = rr.Producer(
    sn.read,  # function that will generate data
    bread,  # output data bus
    0.1,  # delay between data generation cycles
    bTerminate,  # bus to watch for termination signal
    "Read grayscale")


interpret = rr.ConsumerProducer(
    int.process,  # function that will process data
    bread,  # input data buses
    bprocess,  # output data bus
    0.1,  # delay between data control cycles
    bTerminate,  # bus to watch for termination signal
    "Process Data")

control = rr.Consumer(
    con.proportional_control,
    bprocess,
    .1,
    bTerminate,
    "Control")




ulread = rr.Producer(
    usn.read,  # function that will generate data
    buread,  # output data bus
    0.1,  # delay between data generation cycles
    bTerminate,  # bus to watch for termination signal
    "Read grayscale")


ulinterpret = rr.ConsumerProducer(
    uint.process,  # function that will process data
    buread,  # input data buses
    buprocess,  # output data bus
    0.1,  # delay between data control cycles
    bTerminate,  # bus to watch for termination signal
    "Process Data")

ulcontrol = rr.Consumer(
    ucon.control,
    buprocess,
    .1,
    bTerminate,
    "Control")

producer_consumer_list = [read,
                          interpret,
                          control,
                          ulread,
                          ulinterpret,
                          ulcontrol]

rr.runConcurrently(producer_consumer_list)