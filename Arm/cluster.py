# Header
'''
ROB 515: Intro to Robotics II
Final Project
Clement Cantil and Roger Wienaah
Oregon State University
Winter 2025

Based (Heavily) on Code from: https://golsteyn.com/writing/dice
Thanks Quentin!

Golsteyn, Q. (n.d.). A small Python script that reads dice rolls out loud. Retrieved from https://golsteyn.com/writing/dice
'''

#Imports
import sys
sys.path.append('/home/pi/ArmPi/')
import Camera
import cv2
import numpy as np
from sklearn import cluster

class clusterDetection():
    '''
    The pips on the dice are classified as round blobs when viewed from above.
    We filter out non-pip blobs (noise).
    '''
    def __init__(self):
        blobParameters = cv2.SimpleBlobDetector_Params() # Blob detection
        blobParameters.filterByInertia # Filter by Inertia (Roundness of Blobs)
        blobParameters.minInertiaRatio = 0.6 # Varying this to see the effect - Perfectly Circular Is 1; Perfectly Linear Is 0
        self.blobDetector = cv2.SimpleBlobDetector_create(blobParameters) # Create blob detector based on the parameters we've generated
        self.blobs = None
        self.positionList = []
        #self.dice = []
        self.sum = 0
        self.ones = 0
        self.twos = 0
        self.threes = 0
        self.fours = 0
        self.fives = 0
        self.sixes = 0
        self.diceNumUniq = []

    def idBlobs(self, frame):
        blur = cv2.medianBlur(frame, 7) # Replaces pixel values with median value fo neighboring pixel given kernel size
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert to Grayscale
        cv2.imshow('gray', gray)
        self.blobs = self.blobDetector.detect(gray) # Detects Grayscale Blobs
        # Returns Individual Pips
    
    def diceFromBlobs(self): # Apply Clustering to Get Individual Dice
        intermediatePositionList = []
        for item in self.blobs:
            position = item.pt
            if position != None: # If a blob has a valid position
                intermediatePositionList.append(position) # Add to the list
            self.positionList = np.asarray(intermediatePositionList) # Convert to numpy array type
        if len(self.positionList) > 0: # If there are any blobs with valid positions
            clustering = cluster.DBSCAN(eps=40, min_samples=0).fit(self.positionList) # Assigns dots to clusters
            numberOfDice = max(clustering.labels_) + 1 # Add one because Python is a zero index language
            #self.dice = []
            self.diceNumUniq = []
            for item2 in range(numberOfDice): # For each die
                position2 = self.positionList[clustering.labels_ == item2] # Get positions of all dice in a cluster
                centroid = np.mean(position2, axis=0) # Take mean of positions
                #self.dice.append([len(position2), *centroid]) # Unpack iterable centroid data and append with number of blobs in cluster
                self.diceNumUniq.append(len(position2))

    def informationPrintout(self):
        print(self.diceNumUniq)
        self.sum = 0
        self.ones = self.diceNumUniq.count(1)
        self.twos = self.diceNumUniq.count(2)
        self.threes = self.diceNumUniq.count(3)
        self.fours = self.diceNumUniq.count(4)
        self.fives = self.diceNumUniq.count(5)
        self.sixes = self.diceNumUniq.count(6)
        if self.ones > 0:
            print("Number of Ones: ", self.ones)
        if self.twos > 0:
            print("Number of Twos: ", self.twos)
        if self.threes > 0:
            print("Number of Threes: ", self.threes)
        if self.fours > 0:
            print("Number of Fours: ", self.fours)
        if self.fives > 0:
            print("Number of Fives: ", self.fives)
        if self.sixes > 0:
            print("Number of Sixes: ", self.sixes)
            

if __name__ == "__main__":
    pips = clusterDetection()
    
    camera = Camera.Camera()
    camera.camera_open()
    while True:
        frame = camera.frame
        if frame is not None:
            #cv2.imshow('frame', frame)
            pips.idBlobs(frame)
            pips.diceFromBlobs()
            pips.informationPrintout()
            cv2.waitKey(1)
        
        
    
