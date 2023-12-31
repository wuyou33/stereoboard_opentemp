#!/usr/bin/env python
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
import csv
import time
import os
ser = serial.Serial('/dev/ttyUSB0',9600,timeout=None)
frameNumber = 0
saveImages= False
frameNumber = 0
imageFolderName='imagesWalktroughSubstation'
folderExtensionTry=0
while os.path.exists(imageFolderName+str(folderExtensionTry)):
    folderExtensionTry+=1
imageFolderName=imageFolderName+str(folderExtensionTry)
os.makedirs(imageFolderName)

currentBuffer=[]
print cv2.__version__
if '3.0.0'==cv2.__version__:
    cv2.namedWindow('img',cv2.WINDOW_NORMAL)
if '3.0.0-dev'==cv2.__version__:
    cv2.namedWindow('img',cv2.WINDOW_NORMAL)


fileToWrite=file("data.csv",'w')
dataWriter=csv.writer(fileToWrite)

# main loop:
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
            print 'length: ', length, ' count: ', lineCount, ' lineLength: ', lineLength, ' end of images found: ', endOfImagesFound
            if sync1<0:    # We did not find the startbit... try again
                continue

            data = stereoboard_tools.fill_image_array(sync1,oneImage, lineLength, lineCount);
            data=np.array(data);
            data = data[0];
            # print 'u = %d, dist = %d, vx = %f \n' % (data[8], data[4]);
            velocity_x = float(data[8] - 127);
            velocity_y = float(data[9] - 127);
            print 'velocity hor = %f (ver = %f)\n' % (velocity_x, velocity_y)


            
	    #print img
            #img /= 20
            #img /= 6

            # Create a color image
            #img=stereoboard_tools.createRedBlueImage(img,lineCount,lineLength)

            #if (not '3.0.0'==cv2.__version__) and (not '3.0.0-dev'==cv2.__version__):
            #    print 'resizing stuff!'
            #    img = cv2.resize(img,(0,0),fx=20,fy=20,interpolation=cv2.INTER_NEAREST)
            #cv2.imshow('img',img)
	    
            #key=cv2.waitKey(1)
            #if 'q' == chr(key & 255):
            #    break	

            #if saveImages:
            #    stereoboard_tools.saveImages(img, 0, 0, frameNumber, imageFolderName)
            #    frameNumber+=1
            #    print 'saving images'
    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
