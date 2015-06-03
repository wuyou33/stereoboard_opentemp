import numpy as np
import cv2
import serial
import array
import sys
import stereoboard_tools
BAUDRATE=1000000
ser = serial.Serial('/dev/ttyUSB0',BAUDRATE)
size_of_one_image=25348 # 128*96*2+4*96+4*96+4
W = 128
H=96
frameNumber = 0
saveImages=False
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



cv2.namedWindow('img',cv2.WINDOW_NORMAL)
cv2.createTrackbar('offsetLeft','img',maxOffset/2,maxOffset,changeOffsetLeft)
cv2.createTrackbar('offsetRight','img',maxOffset/2,maxOffset,changeOffsetRight)
cv2.createTrackbar('disparityBorder','img',W/2,W,changeOffsetBorder)
raw = []
def createEverythingUsingData():
    try:
        global raw, frameNumber
        # Search for the startposition
        sync, length,lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(raw)
        print 'sync ' , sync, ' ', length, ' ', lineLength, ' ', lineCount

        if sync==0:
            # We did not find the startbit... try again
            return

        img, leftImage, rightImage = stereoboard_tools.fill_image_arrays(raw, sync,size_of_one_image, W, H, DISPARITY_OFFSET_LEFT,DISPARITY_OFFSET_RIGHT,DISPARITY_BORDER)

        # Go from values between 0-255 to intensities between 0.0-1.0
        img /= 255
        leftImage /= 255
        rightImage /=255


        cv2.imshow('img',img)

        key=cv2.waitKey(1000)
        if key > 0:
            print '#define DISPARITY_OFFSET_LEFT ', DISPARITY_OFFSET_LEFT
            print '#define DISPARITY_OFFSET_RIGHT ', DISPARITY_OFFSET_RIGHT
            print '#define DISPARITY_BORDER ', DISPARITY_BORDER

        if saveImages:
            stereoboard_tools.saveImages(img, leftImage, rightImage, frameNumber, 'images')
            frameNumber+=1
    except Exception as lastExcept:
        stereoboard_tools.PrintException()


while True:
    try:
        # Read two times the image size... this way we surely have an image
        raw = ser.read(size_of_one_image*2)
        raw = bytearray(raw)

        createEverythingUsingData()
    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
