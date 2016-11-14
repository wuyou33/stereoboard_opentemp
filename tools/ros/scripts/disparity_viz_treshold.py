import os
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import csv
import time
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
frameNumber = 0
saveImages= True
treshold=0.3
thr=0
nameFolder='imagesOutside'
appendNumber=0
while os.path.isdir(nameFolder+str(appendNumber)):
  appendNumber+=1
nameFolder=nameFolder+str(appendNumber)
os.mkdir(nameFolder)
def nothing(x):
  global thr
  print 'x: ',x, ' old thr: ' , thr
  thr=x

currentBuffer=[]

cv2.namedWindow('img',cv2.WINDOW_NORMAL)
cv2.createTrackbar('threshold','img',0,20,nothing)
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

            totalData=[frameNumber,time.time()]
	    toChangeValues=abs(img.real)>thr
            toChangeValues[img==78]=0
            img.real[toChangeValues] = 30
            #img.real[abs(img.real) > 0]+=1 
	    img+=1
            newImg=img / 20
            # Create a color image
            newImg=stereoboard_tools.createRedBlueImage(newImg,lineCount,lineLength)
            newImg[img==79]=(0,0,0)
            newImg[img==0]=(0,255,0)
            cv2.imshow('img',newImg)
            key=cv2.waitKey(1)
            if saveImages:
                import scipy
                fileNameBoth = nameFolder+'/disparity'+str(frameNumber)+'.png'
		redChannel=newImg[:,:,2].copy()
		newImg[:,:,2]=newImg[:,:,0].copy()
		newImg[:,:,0]=redChannel
		newImg=cv2.resize(newImg,(0,0),fx=3.0,fy=3.0,interpolation=cv2.INTER_NEAREST)
                scipy.misc.imsave(fileNameBoth, newImg)
                frameNumber+=1
	   
    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
