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



def fill_disparity_array(startSync, raw, width, height):
    line=0
    # Initialise image
    img = np.zeros((height,width))
    
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
    return img

currentBuffer=[]
while True:
    try:
        # Read the image
        currentBuffer, location = stereoboard_tools.readPartOfImage(ser, currentBuffer)

        if location > 0:
            oneImage = currentBuffer[0:location]
            currentBuffer=currentBuffer[location::]

            # Search the startbyte
            sync1, length,lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(oneImage)
      #      print 'sync ' , sync1, ' ', length, ' ', lineLength, ' ', lineCount

            if sync1<0:    # We did not find the startbit... try again
                continue


            img = fill_disparity_array(sync1,oneImage, lineLength, lineCount)
            img =img[:,::-1]
            img /= 100

            cv2.namedWindow('img',cv2.WINDOW_NORMAL)
            cv2.imshow('img',img)


            key=cv2.waitKey(1)

            if saveImages:
                import scipy
                fileNameBoth = 'imageBoth'+str(frameNumber)+'.png'
                scipy.misc.imsave(fileNameBoth, img)
                frameNumber+=1

    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep