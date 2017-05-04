#!/usr/bin/env python
import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
import csv
import time

if len(sys.argv) > 1:
  print(sys.argv[1])
  ser = serial.Serial(sys.argv[1],921600,timeout=None)
else:
  ser = serial.Serial('/dev/ttyUSB0',921600,timeout=None)
  
frameNumber = 0
saveImages= False
treshold=0.3
max_time = 500;

currentBuffer=[]
now = time.clock()
cv2.namedWindow('stereoimg',cv2.WINDOW_NORMAL)

# main loop:
while True:
    try:
        # Read the image
        currentBuffer, location, endOfImagesFound = stereoboard_tools.readPartOfImage(ser, currentBuffer)
        startPosition=location[0]
        endPosition=location[1]
        
        if endOfImagesFound > 0:
            oneImage = currentBuffer[startPosition:endPosition]
            currentBuffer = currentBuffer[endPosition::]
            
            # Search the startbyte
            sync1, length, lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(oneImage)
            #print 'length: ', length, ' count: ', lineCount, ' lineLength: ', lineLength, ' end of images found: ', endOfImagesFound
            if sync1 < 0:    # We did not find the startbit... try again
                continue
                
            img = stereoboard_tools.fill_image_array(sync1, oneImage, lineLength, lineCount)
            print img.size
            if img.size <= 32:
              print img
              count = 0
            elif img.size >= 12288:
              if saveImages:
                  import scipy
                  fileNameBoth = 'image'+str(frameNumber)+'.png'
                  scipy.misc.imsave(fileNameBoth, img)
              totalData=[frameNumber,time.time()]
              img *= 2
              
              #print 1 / (time.clock() - now)
              now = time.clock()

              # Create a color image
              img = stereoboard_tools.createRedBlueImage(img,lineCount,lineLength)
              #img = img[::2]
              # if (not '3.0.0'==cv2.__version__) and (not '3.0.0-dev'==cv2.__version__):
              #     print 'resizing stuff!'
              img = cv2.resize(img,(0,0),fx=3,fy=3 ,interpolation=cv2.INTER_NEAREST)
              cv2.imshow('stereoimg',img)

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
