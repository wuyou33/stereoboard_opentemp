#!/usr/bin/env python
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
import csv
import time
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
frameNumber = 0
saveImages= False
treshold=0.3
max_time = 500;

currentBuffer=[]
print cv2.__version__
if '3.0.0'==cv2.__version__:
    cv2.namedWindow('img',cv2.WINDOW_NORMAL)
if '3.0.0-dev'==cv2.__version__:
    cv2.namedWindow('img',cv2.WINDOW_NORMAL)


fileToWrite=file("data.csv",'w')
dataWriter=csv.writer(fileToWrite)

# for velocity estimation:
step = 0
distance = [0]
outlier = 0
time_steps = [0]
plt.axis([0, 128, 0,120])
plt.ion();

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


            img = stereoboard_tools.fill_image_array(sync1,oneImage, lineLength, lineCount)
            img=np.array(img)

            if saveImages:
                import scipy
                fileNameBoth = 'image'+str(frameNumber)+'.png'
                scipy.misc.imsave(fileNameBoth, img)
            totalData=[frameNumber,time.time()]
	    print img


	    far_away_threshold = 12
	    x = 5
	    while img[0,x] > far_away_threshold and x < 125:
	      x += 1


           # img /= 20
           # img /= 6
            plt.clf()
	    plt.axis([0, length, 0,120])
	    plt.plot(range(0,len(img[0,:])),img[0,:])
	    plt.plot([x-1,x,x+1],[0,120,0],'r')
	    plt.draw()
            # Create a color image
            img=stereoboard_tools.createRedBlueImage(img,lineCount,lineLength)

            if (not '3.0.0'==cv2.__version__) and (not '3.0.0-dev'==cv2.__version__):
                print 'resizing stuff!'
                img = cv2.resize(img,(0,0),fx=20,fy=20,interpolation=cv2.INTER_NEAREST)
            cv2.imshow('img',img)
	    	# print 'test hier'

	
            key=cv2.waitKey(1)
            if 'q' == chr(key & 255):
                break	
            if saveImages:
                import scipy
                fileNameBoth = 'imageBoth'+str(frameNumber)+'.png'
                scipy.misc.imsave(fileNameBoth, img)
                frameNumber+=1

    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
