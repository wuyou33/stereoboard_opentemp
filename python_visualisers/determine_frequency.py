#!/usr/bin/env python
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import time
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
frameNumber = 0
saveImages= False



timeTic = time.time()
differenceHistory=[]
currentBuffer=[]
totalEndOfImagesFound=0
readImagesPerFrequencyMeasurement=30
averageOverAmountOfValues=2
while True:
    try:
        # Read the image
        currentBuffer, location,endOfImagesFound = stereoboard_tools.readPartOfImage(ser, currentBuffer)
        if location[0]>-1:
            currentBuffer=currentBuffer[location[1]::]
            totalEndOfImagesFound+=endOfImagesFound
            if totalEndOfImagesFound>readImagesPerFrequencyMeasurement:
                print  'read ', readImagesPerFrequencyMeasurement
                totalEndOfImagesFound=0
                newTimeTic=time.time()
                difference=newTimeTic-timeTic
                timeTic=newTimeTic
                differenceHistory.append(difference)
                sumDifference=0
                for value in differenceHistory:
                    sumDifference+=value

                print 'frequency: ', readImagesPerFrequencyMeasurement*(1/(sumDifference/len(differenceHistory)))
                if len(differenceHistory)>averageOverAmountOfValues:
                     differenceHistory=differenceHistory[-averageOverAmountOfValues::]
    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
