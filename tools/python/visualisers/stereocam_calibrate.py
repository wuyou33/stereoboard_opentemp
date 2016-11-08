#!/usr/bin/env python
import numpy as np
import cv2
import serial
import stereoboard_tools
import array
import sys
import os

W = 128
H=96
DISPARITY_OFFSET_LEFT=0
DISPARITY_OFFSET_RIGHT=0
DISPARITY_BORDER=W/2
previousLeftImage=None
if len(sys.argv) > 1:
  print(sys.argv[1])
  ser = serial.Serial(sys.argv[1],1000000,timeout=None)
else:
  ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)

size_of_one_image=25348 # 128*96*2+4*96+4*96+4
horizontalOffset=0
frameNumber = 0
#imageFolderName='imagesWalktroughSubstation'
#folderExtensionTry=0
#while os.path.exists(imageFolderName+str(folderExtensionTry)):
#    folderExtensionTry+=1
#imageFolderName=imageFolderName+str(folderExtensionTry)
#os.makedirs(imageFolderName)
DISPARITY_OFFSET_LEFT=0
DISPARITY_OFFSET_RIGHT=0
DISPARITY_BORDER=W/2
maxOffset = 16

def changeOffsetLeft(newValue):
    global  DISPARITY_OFFSET_LEFT
    DISPARITY_OFFSET_LEFT = newValue-maxOffset/2

def changeOffsetRight(newValue):
    global  DISPARITY_OFFSET_RIGHT
    DISPARITY_OFFSET_RIGHT= newValue-maxOffset/2

def changeOffsetBorder(newValue):
    global  DISPARITY_BORDER
    DISPARITY_BORDER= newValue


def changeOffsetHorizontal(newValue):
    global  horizontalOffset
    horizontalOffset= newValue - maxOffset/2


cv2.namedWindow('img', cv2.WINDOW_NORMAL)
cv2.createTrackbar('offsetLeft','img',maxOffset/2,maxOffset,changeOffsetLeft)
cv2.createTrackbar('offsetRight','img',maxOffset/2,maxOffset,changeOffsetRight)
cv2.createTrackbar('disparityBorder','img',W/2,W,changeOffsetBorder)
cv2.createTrackbar('horizontalOffset','img',maxOffset/2,maxOffset,changeOffsetHorizontal)

currentBuffer=[]
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

            if sync1<0:    # We did not find the startbit... try again
                continue


            # Fill the image arrays
            img, leftImage, rightImage = stereoboard_tools.fill_image_arrays(
                oneImage, sync1, size_of_one_image, W, H, DISPARITY_OFFSET_LEFT,DISPARITY_OFFSET_RIGHT,DISPARITY_BORDER,horizontalOffset)

            # Go from values between 0-255 to intensities between 0.0-1.0
            img /= 255

            # Show the images
            cv2.imshow('img',img)
            key=cv2.waitKey(1)
            print '-----------------------------------------------------'
	    print '#define DISPARITY_OFFSET_LEFT ', DISPARITY_OFFSET_LEFT
	    print '#define DISPARITY_OFFSET_RIGHT ', DISPARITY_OFFSET_RIGHT
	    print '#define DISPARITY_BORDER ', DISPARITY_BORDER
	    print '#define DISPARITY_OFFSET_HORIZONTAL ', horizontalOffset

    except KeyboardInterrupt:
	break
    except Exception:
        print 'error!'
        stereoboard_tools.PrintException()

