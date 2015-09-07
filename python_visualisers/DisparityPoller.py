import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import csv
import time

class DisparityPoller:
    """Polls the stereoboard"""

    def pollStereoboard(self):
        # Read the image
        currentBuffer, location,endOfImagesFound = stereoboard_tools.readPartOfImage(ser, currentBuffer)
        startPosition=location[0]
        endPosition=location[1]
        if location[0] > -1:
            oneImage = currentBuffer[startPosition:endPosition]
            currentBuffer=currentBuffer[endPosition::]
            
            
            # Search the startbyte
            sync1, length,lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(oneImage)
            if self.printUpdates:
                print 'length: ', length, ' count: ', lineCount, ' lineLength: ', lineLength, ' end of images found: ', endOfImagesFound
            if sync1<0:    # We did not find the startbit... try again later
                return
        
        
            img = stereoboard_tools.fill_image_array(sync1,oneImage, lineLength, lineCount)
            self.lastImg=np.array(img)
            self.lastImgTimestamp=time.time()
    def __init__(self, serialPortName, baudRate,printUpdates=False):
        self.ser = serial.Serial(serialPortName,baudRate,timeout=None)
        self.currentBuffer=[]
        self.lastImgTimestamp=None
        self.printUpdates=printUpdates


