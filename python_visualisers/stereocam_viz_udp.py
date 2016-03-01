#!/usr/bin/env python
import numpy as np
import sys

sys.path.append('/usr/local/lib/python2.7/site-packages')

import cv2
import serial
import stereoboard_tools
import array
import sys
import os
import socket
UDP_IP = ""
UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
print "started something"

img = np.zeros((96,256))

while True:
    data, addr = sock.recvfrom(512) # buffer size is 1024 bytes
    # print "received message:", data
    raw = bytearray(data)
    currentBuffer = []
    # Add all bytes to our total collection of bytes

    for byte in raw:
        currentBuffer.append(int(byte))
        # print 'read: ', int(byte)
    if currentBuffer[0] < img.shape[0] and len(currentBuffer) == img.shape[1]:
        # print 'read: ', currentBuffer
        img[currentBuffer[0],:]=currentBuffer
        img[currentBuffer[0],:]/=256
        img2 = cv2.resize(img,(0,0),fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
        cv2.imshow('img',img2)
        cv2.waitKey(1)
    else:
        print 'shape: ', len(currentBuffer), "ssfsf",img.shape[1]
        print 'sfsf;,',img.shape[0]
        print 'true?  ',currentBuffer[0] < img.shape[0]
        print 'second: ' , len(currentBuffer) == img.shape[1]
        print currentBuffer
#
# BAUDRATE=115200
# W = 128
# H=96
# saveImages=False
# DISPARITY_OFFSET_LEFT=0

# DISPARITY_OFFSET_RIGHT=0
# DISPARITY_BORDER=W/2
# previousLeftImage=None
# ser = serial.Serial('/dev/tty.usbserial-FTHHXIFQ',BAUDRATE)
# size_of_one_image=25348 # 128*96*2+4*96+4*96+4
#
# frameNumber = 0
# if saveImages:
#     imageFolderName='imagesWalktroughSubstation'
#     folderExtensionTry=0
#     while os.path.exists(imageFolderName+str(folderExtensionTry)):
#         folderExtensionTry+=1
#     imageFolderName=imageFolderName+str(folderExtensionTry)
#     os.makedirs(imageFolderName)
#
# cv2.namedWindow('img', cv2.WINDOW_NORMAL)
# cv2.namedWindow('leftimg', cv2.WINDOW_NORMAL)
# cv2.namedWindow('rightimg', cv2.WINDOW_NORMAL)
#
# currentBuffer=[]
# while True:
#     try:
#         # Read the image
#         currentBuffer, location,endOfImagesFound = stereoboard_tools.readPartOfImage(ser, currentBuffer)
#         startPosition=location[0]
#         endPosition=location[1]
#         if location[0] > -1:
#             oneImage = currentBuffer[startPosition:endPosition]
#             currentBuffer=currentBuffer[endPosition::]
#
#             # Search the startbyte
#             sync1, length,lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(oneImage)
#
#             if sync1<0:    # We did not find the startbit... try again
#                 continue
#
#
#             # Fill the image arrays
#             img, leftImage, rightImage = stereoboard_tools.fill_image_arrays(
#                 oneImage, sync1,size_of_one_image, W, H, DISPARITY_OFFSET_LEFT,DISPARITY_OFFSET_RIGHT,DISPARITY_BORDER,0)
#
#             # Go from values between 0-255 to intensities between 0.0-1.0
#             img /= 255
#             leftImage /= 255
#             rightImage /= 255
#             # Show the images
#             cv2.imshow('img',img)
#             cv2.imshow('leftimg',leftImage)
#             cv2.imshow('rightimg',rightImage)
#             key=cv2.waitKey(1)
#             print 'done'
#             if saveImages:
#                 stereoboard_tools.saveImages(img, leftImage, rightImage, frameNumber, imageFolderName)
#                 frameNumber+=1
#     except KeyboardInterrupt:
# 	break
#     except Exception:
#         print 'error!'
#         stereoboard_tools.PrintException()
#
