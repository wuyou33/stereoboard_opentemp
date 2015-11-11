#!/usr/bin/env python
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import csv
import os

import matplotlib.pyplot as plt
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
frameNumber = 0
saveImages= False
proximityHistory=[]
nameRecording="proximityData/SubstationEverywhereCollisions"
nameRecordingNumber=1;
while os.path.exists(nameRecording+str(nameRecordingNumber)):
    nameRecordingNumber+=1
nameRecording=nameRecording+str(nameRecordingNumber)
writeFile=file(nameRecording,'w')
writer=csv.writer(writeFile)
writer.writerow(["REGISTER_PDATA","REGISTER_PDATA2","CDATAL","CDATAH"])

currentBuffer=[]
plt.ion()
#cv2.namedWindow('img',cv2.WINDOW_NORMAL)

while True:
    try:
        # Read the image
        currentBuffer, location,endOfImagesFound = stereoboard_tools.readPartOfImage(ser, currentBuffer)
        startPosition=location[0]
        endPosition=location[1]
        if location[0] > -1:
            oneImage = currentBuffer[startPosition:endPosition]
            currentBuffer=currentBuffer[endPosition::]


            # Search the startbyte
            sync1, length,lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(oneImage)
            #print 'length: ', length, ' count: ', lineCount, ' lineLength: ', lineLength, ' end of images found: ', endOfImagesFound
            if sync1<0:    # We did not find the startbit... try again
                continue


            img = stereoboard_tools.fill_image_array(sync1,oneImage, lineLength, lineCount)
            img=np.array(img)
            data=[]
            data.append(img[0,:])
            writer.writerow(data)

            proximityHistory.append(img[0,0])
            plt.plot()

            plt.draw()
            print data

            if saveImages:
                import scipy
                fileNameBoth = 'imageBoth'+str(frameNumber)+'.png'
                scipy.misc.imsave(fileNameBoth, img)
                frameNumber+=1

    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
