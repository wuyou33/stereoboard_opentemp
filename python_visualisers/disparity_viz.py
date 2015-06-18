import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
frameNumber = 0
saveImages= False
currentBuffer=[]
maxExpectedInImage=20 # Each pixel is divided by this value to go to grayscale values for the cv2 image
cv2.namedWindow('img',cv2.WINDOW_NORMAL)

while True:
    try:
        # Read the image
        currentBuffer, location = stereoboard_tools.readPartOfImage(ser, currentBuffer)

        if location > 0:
            oneImage = currentBuffer[0:location]
            currentBuffer=currentBuffer[location::]

            # Search the startbyte
            sync1, length,lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(oneImage)
            print 'length: ', length, ' count: ', lineCount, ' lineLength: ', lineLength
            if sync1<0:    # We did not find the startbit... try again
                continue


            img = stereoboard_tools.fill_image_array(sync1,oneImage, lineLength, lineCount)
            img=np.array(img)
            img /= maxExpectedInImage
         #   stereoboard_tools.draw_sonar_visualisation(img, img.shape[0])


          #  img=cv2.cvtColor(img, img, cv2.COLOR_GRAY2BGR)

            cv2.imshow('img',img)


            key=cv2.waitKey(100)

            if saveImages:
                import scipy
                fileNameBoth = 'imageBoth'+str(frameNumber)+'.png'
                scipy.misc.imsave(fileNameBoth, img)
                frameNumber+=1

    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep