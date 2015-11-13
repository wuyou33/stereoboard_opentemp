#!/usr/bin/env python
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
import csv
import time
ser = serial.Serial('/dev/ttyUSB0',9600,timeout=None)
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
step = 0;
avg_disparity = [0];
time_steps = [0];
plt.axis([0, 1000, 0, 1]);
plt.ion();
plt.show();
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
	    	# print img
            img /= 20
            img /= 6
           

            # keep a time step list and average disparity list for plotting:
            step += 1;
            time_steps.extend([step]);
            # for now maximum disparity, later the average:
            max_disparity = np.max(img[:,:]);
            avg_disparity.extend([max_disparity]);
            if(len(avg_disparity) > max_time):
                avg_disparity.pop(0);
            if(len(time_steps) > max_time):
                time_steps.pop(0);            
            if(np.mod(step, 5) == 0):
                t = np.array(time_steps);
                d = np.array(avg_disparity);
                plt.clf();
                plt.plot(t, d);
                plt.draw()

            print 'last element:', avg_disparity[-1]

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
