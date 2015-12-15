#!/usr/bin/env python
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
import csv
import time
ser = serial.Serial('/dev/ttyUSB0',115200,timeout=None)
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
step = 0;
distance = [0];
outlier = 0;
time_steps = [0];
plt.axis([0, 1000, 0, 1]);
plt.ion();
plt.show();

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
            img /= 20
            img /= 6
           

            # *** Disparity based velocity estimate ***
            # keep a time step list and average disparity list for plotting:
            step += 1;
            # for now maximum disparity, later the average:
            max_disparity = np.max(img[:,:]);
            dist = 1.0 / (max_disparity + 0.1);
            alpha = 0.95;
            new_dist = alpha*distance[-1] + (1-alpha)*dist;
            # Deal with outliers:
            # Single outliers are discarded, while persisting outliers will lead to an array reset:
            MAX_SUBSEQUENT_OUTLIERS = 5;
            if(np.abs(dist - distance[-1]) > 1.5):
                outlier+=1;
                if(outlier >= MAX_SUBSEQUENT_OUTLIERS):
                    # The drone has probably turned in a new direction:
                    print '*** TURNED!!! ***'
                    distance = [dist];
                    time_steps = [step];
                    outlier = 0;
            else:
                outlier = 0;
                # append:
                time_steps.extend([step]);
                distance.extend([new_dist]);
            
            # determine velocity (very simple method):
            n_steps_velocity = 20;
            if(len(distance) > n_steps_velocity):
                velocity = distance[-n_steps_velocity] - distance[-1];
                print 'Velocity = ', velocity

            # keep maximum array size:
            if(len(distance) > max_time):
                distance.pop(0);
            if(len(time_steps) > max_time):
                time_steps.pop(0);    

            # plot the arrays every 5 time steps:        
            if(np.mod(step, 5) == 0):
                t = np.array(time_steps);
                d = np.array(distance);
                plt.clf();
                plt.plot(t, d);
                plt.draw()

            print 'len(distance) = ', len(distance) , ' len(time_steps) = ', len(time_steps) ,'last element:', distance[-1]

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
