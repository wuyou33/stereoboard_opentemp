import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import csv
import time
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
frameNumber = 0
saveImages= False
treshold=0.3

currentBuffer=[]
if '3.0.0'==cv2.__version__:
    cv2.namedWindow('img',cv2.WINDOW_NORMAL)

fileToWrite=file("data.csv",'w')
dataWriter=csv.writer(fileToWrite)
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
            img /= 20
            img /= 6

            # Create a color image
            img=stereoboard_tools.createRedBlueImage(img,lineCount,lineLength)

            if not '3.0.0'==cv2.__version__:
                img = cv2.resize(img,(0,0),fx=20,fy=20,interpolation=cv2.INTER_NEAREST)
            cv2.imshow('img',img)


            key=cv2.waitKey(1)
            if 'q' == chr(key & 255):
                break	
            if 'a' == chr(key & 255):
                stereoboard_tools.stereoboardCommand=1	
            if 's' == chr(key & 255):
                stereoboard_tools.stereoboardCommand=2
            if 'd' == chr(key & 255):
                stereoboard_tools.stereoboardCommand=3
            if 'f' == chr(key & 255):
                stereoboard_tools.stereoboardCommand=4
            if 'z' == chr(key & 255):
                stereoboard_tools.stereoboardCommand=5
            if saveImages:
                import scipy
                fileNameBoth = 'imageBoth'+str(frameNumber)+'.png'
                scipy.misc.imsave(fileNameBoth, img)
                frameNumber+=1

    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
