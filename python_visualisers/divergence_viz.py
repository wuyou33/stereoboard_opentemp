import cv2
import serial
import stereoboard_tools
import numpy as np
import matplotlib.pyplot as plt
import time
plt.axis([0,500, 0, 200])
plt.ion()
plt.show()
   
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
frameNumber = 0
saveImages= False
currentBuffer=[]
maxExpectedInImage=250 # Each pixel is divided by this value to go to grayscale values for the cv2 image
cv2.namedWindow('img',cv2.WINDOW_NORMAL)
yintHistory=[]
while True:
    try:
        # Read the image
        currentBuffer, location = stereoboard_tools.readDivergenceFromSerial(ser, currentBuffer)
        startPosition=location[0]
        endPosition=location[1]
        if location[0] > 0:
            oneImage = currentBuffer[startPosition:endPosition]
            currentBuffer=currentBuffer[endPosition::]

            # Search the startbyte
            sync1, length,lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(oneImage)
            #print 'length: ', length, ' count: ', lineCount, ' lineLength: ', lineLength
            if sync1<0:    # We did not find the startbit... try again
                continue


            img = stereoboard_tools.fill_image_array(sync1,oneImage, lineLength, lineCount)
            img=np.array(img)
	    slope=img[0,0]
            yint=img[0,1]
            print 'sloap: ' , slope, ' yint: ' , yint
	    print img
            yintHistory.append(yint)
	    plt.plot(yintHistory)
	    plt.draw()
	    time.sleep(0.05)
        

    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
