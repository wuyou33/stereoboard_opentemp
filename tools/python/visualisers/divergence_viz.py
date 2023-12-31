#!/usr/bin/env python
import cv2
import serial
import stereoboard_tools
import numpy as np
import matplotlib.pyplot as plt
import time
import math
plt.axis([0,300, 0 , 255])
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
velocity_x_pixelwiseHistory=[]
velocity_z_pixelwiseHistory=[]

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
	    slope_x=(img[0,0]-100)/1000
            yint_x=(img[0,1]-100)/100
	    slope_y=(img[0,2]-100)/1000
            yint_y=(img[0,3]-100)/100
            height=img[0,4]

            velocity_x_stereoboard = (img[0,18]-127)/100
            velocity_y_stereoboard =(img[0,19]-127)/100
            velocity_x_pixelwise_stereoboard = (img[0,20]-127)/100
            velocity_z_pixelwise_stereoboard =(img[0,21]-127)/100
            #velocity_y_stereoboard = (img[0,9]-127)/100


            velocity_xHistory.append(velocity_x_stereoboard)
            velocity_yHistory.append(velocity_y_stereoboard)
            velocity_x_pixelwiseHistory.append(velocity_x_pixelwise_stereoboard)
            velocity_z_pixelwiseHistory.append(velocity_z_pixelwise_stereoboard)
            #velocity_xHistory.append(img[0,15])            
            #velocity_yHistory.append(img[0,16])            

            if counter > 100:
                del velocity_xHistory[0]
            	del velocity_yHistory[0]
                del velocity_x_pixelwiseHistory[0]
            	del velocity_z_pixelwiseHistory[0]
            else:
                counter = counter + 1

            plt.cla()
	    plt.axis([0,100, -1,1])
	   # plt.plot(velocity_xHistory)
	    plt.draw()
           #plt.plot(velocity_yHistory)
	    plt.draw()
	    plt.plot(velocity_x_pixelwiseHistory)
	    plt.draw()
            plt.plot(velocity_z_pixelwiseHistory)
	    plt.draw()


	    #time.sleep(0.05)
        
            
    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
