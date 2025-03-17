# Header
'''
ROB 515: Intro to Robotics II
Final Project
Clement Cantil and Roger Wienaah
Oregon State University
Winter 2025

Based (Heavily) on Code from: https://www.geeksforgeeks.org/text-detection-and-extraction-using-opencv-and-ocr/

GeeksforGeeks. "Text Detection and Extraction Using OpenCV and OCR." GeeksforGeeks, https://www.geeksforgeeks.org/text-detection-and-extraction-using-opencv-and-ocr/.
'''

# Imports
import cv2
import pytesseract

class textReader():
    def __init__(self):
        pytesseract.pytesseract.tesseract_cmd = "~/ArmPi/Functions/venv/bin/pytesseract"
        
    def processing(frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert image to grayscale
        _, thresh1 = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU | cv2.THRESH_BINARY_INV) # Apply OTSU Thresholding (Bimodal)
        rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (18, 18)) # Area surrounding elements
        dilation = cv2.dilate(thresh1, rect_kernel, iterations=1) # Dilate
        contours, _ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # Contouring
        im2 = frame.copy
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            _ = cv2.rectangle(im2, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cropped = im2[y:y + h, x:x + w]
            text = pytesseract.image_to_string(cropped)
            print(text)

if __name__ == "__main__":
    textreader = textReader()
    
    camera = Camera.Camera()
    camera.camera_open()
    while True:
        frame = camera.frame
        if frame is not None:
            cv2.imshow('frame', frame)
            textreader.processing(frame)
            cv2.waitKey(1)

