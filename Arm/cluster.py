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
        blobParameters.minEnertiaRatio = 0.7 # Varying this to see the effect - Perfectly Circular Is 1; Perfectly Linear Is 0
        self.blobDetector = cv2.SimpleBlobDetector_create(blobParameters) # Create blob detector based on the parameters we've generated
        self.blobs = None
        self.positionList = []
        self.dice = []
        self.sum = 0
        self.ones = 0
        self.twos = 0
        self.threes = 0
        self.fours = 0
        self.fives = 0
        self.sixes = 0

    def idBlobs(self, frame):
        blur = cv2.medianBlur(frame, 7) # Replaces pixel values with median value fo neighboring pixel within kernel of size 7
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY) # Convert to Grayscale
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
            self.dice = []
            for item2 in range(numberOfDice): # For each die
                position2 = self.positionList[clustering.labels_ == item2] # Get positions of all dice in a cluster
                centroid = np.mean(position2, axis=0) # Take mean of positions
                self.dice.append([len(position2), *centroid]) # Unpack iterable centroid data and append with number of blobs in cluster

    def informationPrintout(self):
        self.sum = 0
        self.ones = self.dice.count(1)
        self.twos = self.dice.count(2)
        self.threes = self.dice.count(3)
        self.fours = self.dice.count(4)
        self.fives = self.dice.count(5)
        self.sixes = self.dice.count(6)
        if self.ones > 0:
            print("Number of Ones: ", self.ones)
        if self.twos > 0:
            print("Number of Twos: ", self.ones)
        if self.threes > 0:
            print("Number of Threes: ", self.ones)
        if self.fours > 0:
            print("Number of Fours: ", self.ones)
        if self.fives > 0:
            print("Number of Fives: ", self.ones)
        if self.sixes > 0:
            print("Number of Sixes: ", self.ones)