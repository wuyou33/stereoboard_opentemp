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


currentBuffer=[]

#cv2.namedWindow('img',cv2.WINDOW_NORMAL)
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
                fileNameBoth = 'imageBoth'+str(frameNumber)+'.png'
                scipy.misc.imsave(fileNameBoth, img)
		totalData=[frameNumber,time.time()]
		for row in img:
		    totalData.extend(row)
		dataWriter.writerow(totalData)
                frameNumber+=1
            if lineCount < 20:
            	stereoboard_tools.draw_sonar_visualisation(img, img.shape[0])
            	img /= 20


            	# Create a color image
            	img=stereoboard_tools.createRedBlueImage(img,lineCount,lineLength)


          #  img=cv2.cvtColor(img, img, cv2.COLOR_GRAY2BGR)
	    else:
		    # Fill the image arrays
		    size_of_one_image=25348 # 128*96*2+4*96+4*96+4
		    W = 128
	            H=96
		    DISPARITY_OFFSET_LEFT=0
		    DISPARITY_OFFSET_RIGHT=0
		    DISPARITY_BORDER=W/2
	
		    img, leftImage, rightImage = stereoboard_tools.fill_image_arrays(
		        oneImage, sync1,size_of_one_image, W, H, DISPARITY_OFFSET_LEFT,DISPARITY_OFFSET_RIGHT,DISPARITY_BORDER)

		    # Go from values between 0-255 to intensities between 0.0-1.0
		    img /= 255
		    leftImage /= 255
		    rightImage /= 255
		    # Show the images
		    cv2.imshow('img',img)
		    cv2.imshow('leftimg',leftImage)
		    cv2.imshow('rightimg',rightImage)
		    key=cv2.waitKey(1)

		    if saveImages:
		        stereoboard_tools.saveImages(img, leftImage, rightImage, frameNumber, 'images')
		        frameNumber+=1
	    img/=100

            cv2.imshow('img',img)


            key=cv2.waitKey(1)


    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
