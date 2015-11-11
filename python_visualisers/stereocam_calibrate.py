import numpy as np
import cv2
import serial
import stereoboard_tools
import array
import sys
import os

BAUDRATE=1000000
W = 128
H=96
DISPARITY_OFFSET_LEFT=0
DISPARITY_OFFSET_RIGHT=0
DISPARITY_BORDER=W/2
previousLeftImage=None
ser = serial.Serial('/dev/ttyUSB0',BAUDRATE)
size_of_one_image=25348 # 128*96*2+4*96+4*96+4

frameNumber = 0
imageFolderName='imagesWalktroughSubstation'
folderExtensionTry=0
while os.path.exists(imageFolderName+str(folderExtensionTry)):
    folderExtensionTry+=1
imageFolderName=imageFolderName+str(folderExtensionTry)
os.makedirs(imageFolderName)
DISPARITY_OFFSET_LEFT=0
DISPARITY_OFFSET_RIGHT=0
DISPARITY_BORDER=W/2
maxOffset = 16

def changeOffsetLeft(newValue):
    global  DISPARITY_OFFSET_LEFT
    DISPARITY_OFFSET_LEFT = newValue-maxOffset/2
    createEverythingUsingData()

def changeOffsetRight(newValue):
    global  DISPARITY_OFFSET_RIGHT
    DISPARITY_OFFSET_RIGHT= newValue-maxOffset/2
    createEverythingUsingData()

def changeOffsetBorder(newValue):
    global  DISPARITY_BORDER
    DISPARITY_BORDER= newValue
    createEverythingUsingData()




cv2.namedWindow('img', cv2.WINDOW_NORMAL)
cv2.createTrackbar('offsetLeft','img',maxOffset/2,maxOffset,changeOffsetLeft)
cv2.createTrackbar('offsetRight','img',maxOffset/2,maxOffset,changeOffsetRight)
cv2.createTrackbar('disparityBorder','img',W/2,W,changeOffsetBorder)
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
                oneImage, sync1,size_of_one_image, W, H, DISPARITY_OFFSET_LEFT,DISPARITY_OFFSET_RIGHT,DISPARITY_BORDER)

            # Go from values between 0-255 to intensities between 0.0-1.0
            img /= 255
            img = cv2.Laplacian(img,cv2.CV_64F)
            img = cv2.Sobel(img,cv2.CV_64F,0,1,ksize=3)

            # Show the images
            cv2.imshow('img',img)
            key=cv2.waitKey(1)
            if key > 0:
		    print '#define DISPARITY_OFFSET_LEFT ', DISPARITY_OFFSET_LEFT
		    print '#define DISPARITY_OFFSET_RIGHT ', DISPARITY_OFFSET_RIGHT
		    print '#define DISPARITY_BORDER ', DISPARITY_BORDER

    except KeyboardInterrupt:
	break
    except Exception:
        print 'error!'
        stereoboard_tools.PrintException()

