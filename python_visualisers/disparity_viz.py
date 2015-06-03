import numpy as np
import cv2
import serial
import matplotlib.pyplot as plt
import stereoboard_tools
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
size_of_one_image=25348
W = 128
H=96
frameNumber = 0
saveImages= False



def fill_disparity_array(startSync):
    global i, startOfBuf, endOfBuf, line
    # Fill the image arrays
    for i in range(startSync + 4, (size_of_one_image / 2) + startSync, 136):
        if (raw[i] == 255) and (raw[i + 1] == 0) and (raw[i + 2] == 0):
            if (raw[i + 3] == 128):
                # print i
                # Start Of Line
                startOfBuf = i + 4
                endOfBuf = (i + 4 + 128)

                img[line, :] = raw[startOfBuf:endOfBuf]
                line += 1;
                ## START MATRIX
                # Search for the startposition

previousLeftImage = None
while True:
    # Read the image
    raw = ser.read(size_of_one_image*2)    # Read two times the image size... this way we surely have an image
    raw = bytearray(raw)

    # Initialise image
    img = np.zeros((H, W))

    # Initialise the startposition in the buffer and the linenumber
    sync=0
    line =0

    # Search the startbyte
    sync1, length, width, height = stereoboard_tools.determine_image_and_line_length(raw)
    print 'sync 1 ' , sync, ' length: ', length, ' width: ', width, ' height: ', height
    if sync1<0:    # We did not find the startbit... try again
        continue

    fill_disparity_array(sync1)
    img =img[:,::-1]
    img /= 100
    if previousLeftImage!=None:
            # print 'sumLeftImage: ', np.sum(leftImage)
            # print 'sum previous Image: ', np.sum(previousLeftImage)
            diffImage = previousLeftImage-img
            cv2.imshow('img',diffImage)
            print 'sum difference: ', np.sum(diffImage)
    previousLeftImage = img

    cv2.namedWindow('img',cv2.WINDOW_NORMAL)
    # cv2.imshow('img',img)


    key=cv2.waitKey(1)

    if saveImages:
        import scipy
        fileNameBoth = 'imageBoth'+str(frameNumber)+'.png'
        scipy.misc.imsave(fileNameBoth, img)
        frameNumber+=1

