import cv2
import numpy as np
from vilib import Vilib
from _picarx_improved_ import Picarx
from sensor_classes_w3 import CONTROL
import time
import ROSS as rr
import logging
# logging.getLogger().setLevel(logging.DEBUG)
logging.getLogger().setLevel(logging.INFO)
px = Picarx()

#Line following functions:
def sensor_cam():

    Vilib.camera_start()
    Vilib.display()
    time.sleep(0.2)
    t = 1
   
    name = f"image{t}"  
    path = "picarx"

    status = Vilib.take_photo(name, path)
    if status:
        full_path = f"{path}/{name}.jpg"
        if Vilib.img is not None and isinstance(Vilib.img, np.ndarray):
            cv2.imwrite(full_path, Vilib.img)  # Save the image
            t += 1
            frame = cv2.imread(f'{path}/{name}.jpg')
            return frame 
        else:
            print("Image not valid")

def interp_cam(frame):

    if frame is None:
        print("No frame available")
          # Sleep briefly to avoid busy waiting

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    #Get binary image
    _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
    
    #Focus on bottom half of image
    frame_height = frame.shape[0]
    frame_width = frame.shape[1]

    lower_half = binary[frame_height//2:,:]
    #Find contours
    contours, _ = cv2.findContours(lower_half, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    center_line = max(contours, key = cv2.contourArea)

    
    # Find centroid point of the largest contour
    centroid = cv2.moments(center_line)
    if centroid['m00'] !=0:
        x_center = int(centroid['m10'] / centroid['m00'])
        x_ratio = (x_center - frame_width / 2) / (frame_width / 2) #Get x in terms of -1 to 1
        # Draw a red dot at the x_center position
        # cv2.circle(frame, (x_center, frame.shape[0] // 2), 5, (0, 0, 255), -1)  # Red dot
        # # Save the modified image
        # cv2.imwrite(f'{path}/{name}.jpg', frame)
    if x_ratio is not None:
        return x_ratio
    else:
        print("x was None")

def angle_controller(x):
    C = CONTROL(px, scale_factor = 30)
        
    #print("X RATIO CONTROLLER YIPPIE:", x)
    xval = -1*x
    C.correct_car(xval)

    #px.forward(25)

#Ultrasonic sensor functions:

#1. Read sensor
def sonic_sensor():
    distance = round(px.ultrasonic.read(), 2)
    return distance

#2. Decide to move px.forward or px.stop based on sensor reading
def sonic_stop(distance):
    safe_d = 40
    speed = 25
    if distance >= safe_d:
        px.forward(speed)
    elif distance < safe_d:
        px.stop
    else:
        px.forward(speed)

    
#Create wrapper for each of the ultrasonicconsumers/producers like in the example code, 
# then when you create the list at the end include everything
#Then, use rr.run_concurrently with that comprehensive list as the input


#Create buses, 2 for the control, 1 for ultrasonic
cam_bus = rr.Bus(sensor_cam(), "Sensor out bus")
interp_bus = rr.Bus(interp_cam(), "Interp out bus")
ultrasonic_bus = rr.Bus(sonic_sensor(), "Ultrasonic out bus")
bTerminate = rr.Bus(0, "Termination Bus")

#Create the wrappers for each producer/consumer

#Camera thread 

camera_write = rr.Producer( sensor_cam,  # function that will generate data
    cam_bus,  # output data bus
    0.05,  # delay between data generation cycles
    bTerminate,  # bus to watch for termination signal
    "Read camera frame")
interp_readwrite = rr.ConsumerProducer(
    interp_cam,  # function that will process data
    cam_bus,  # input data bus
    interp_bus,  # output data bus
    0.05,  # delay between data control cycles
    bTerminate,  # bus to watch for termination signal
    "Read cam, write x")
anglecontrol_read = rr.Consumer(
    angle_controller,  # function that will process data
    interp_bus,  # input data bus
    0.05,  # delay between data control cycles
    bTerminate,  # bus to watch for termination signal
    "Read x, control car")

#Sonic sensor thread

sonic_write = rr.Producer(sonic_sensor,  # function that will generate data
    ultrasonic_bus,  # output data bus
    0.05,  # delay between data generation cycles
    bTerminate,  # bus to watch for termination signal
    "Write sonic sensor")

sonic_read = rr.Consumer(
    sonic_stop,  # function that will process data
    ultrasonic_bus,  # input data bus
    0.05,  # delay between data control cycles
    bTerminate,  # bus to watch for termination signal
    "Read sonic, stop car?")




# Make a printer that returns the most recent wave and product values
printBuses = rr.Printer(
    (cam_bus, interp_bus, ultrasonic_bus, bTerminate),  # input data buses
    # bMultiplied,      # input data buses
    0.25,  # delay between printing cycles
    bTerminate,  # bus to watch for termination signal
    "Print raw and derived data",  # Name of printer
    "Data bus readings are: ")  # Prefix for output

# Make a timer (a special kind of producer) that turns on the termination
# bus when it triggers
terminationTimer = rr.Timer(
    bTerminate,  # Output data bus
    3,  # Duration
    0.01,  # Delay between checking for termination time
    bTerminate,  # Bus to check for termination signal
    "Termination timer")  # Name of this timer


# Create a list of producer-consumers to execute concurrently
producer_consumer_list = [camera_write,
                          interp_readwrite,
                          anglecontrol_read,
                          sonic_write,
                          sonic_read
                        ]

# Execute the list of producer-consumers concurrently
rr.runConcurrently(producer_consumer_list) 

        
