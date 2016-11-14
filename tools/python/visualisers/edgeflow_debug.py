#!/usr/bin/env python
import cv2
import serial
import stereoboard_tools
import numpy as np
import matplotlib.pyplot as plt
import time
import math
plt.axis([0,128, 0 , 255])
plt.ion()
plt.show()
   
ser = serial.Serial('/dev/ttyUSB0',115200,timeout=None)
height = 0
radperpx = 0
distance_pinhole = 0.03
angle_disp = 0
saveImages= False
currentBuffer=[]
FOV_x=57.4
FOV_y=45
maxExpectedInImage=250 # Each pixel is divided by this value to go to grayscale values for the cv2 image
cv2.namedWindow('img',cv2.WINDOW_NORMAL)
velocity_x=0
velocity_y=0
velocity_xHistory=[]
velocity_yHistory=[]

counter = 0;
while True:
    try:
        # Read the image
        currentBuffer, location = stereoboard_tools.readDivergenceFromSerial(ser, currentBuffer)
        startPosition=location[0]
        endPosition=location[1]
        if location[0] > 0:
            oneImage = currentBuffer[startPosition:endPosition]
	    
            currentBuffer=currentBuffer[endPosition::]

            # Search the startbyte
            sync1, length,lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(oneImage)
            #print 'length: ', length, ' count: ', lineCount, ' lineLength: ', lineLength
            if sync1<0:    # We did not find the startbit... try again
                continue


            img = stereoboard_tools.fill_image_array(sync1,oneImage, lineLength, lineCount)
	    print 'img: ' , img
            img=np.array(img)
 	    img_t = np.transpose(img)

            plt.cla()

	    plt.axis([0,128, 0 , 255])

            plt.plot(img_t)
	    plt.draw()


	    time.sleep(0.2)
        
            
    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
