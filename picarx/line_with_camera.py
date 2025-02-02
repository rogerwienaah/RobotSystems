import numpy as np
import cv2
from picarx_improved import Picarx
from picamera2 import Picamera2
import time



picam2 = Picamera2()


camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
picam2.start()


while True:
    img = picam2.capture_array()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cv2.imshow("PiCam", img)
    cv2.waitKey(1)


