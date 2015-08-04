import cv2
import serial
import stereoboard_tools
import numpy as np
import matplotlib.pyplot as plt
import time
plt.axis([0,300, 0, 200])
plt.ion()
plt.show()
   
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
frameNumber = 0
saveImages= False
currentBuffer=[]
maxExpectedInImage=250 # Each pixel is divided by this value to go to grayscale values for the cv2 image
cv2.namedWindow('img',cv2.WINDOW_NORMAL)
yint_yHistory=[]
yint_xHistory=[]
slope_xHistory=[]
slope_yHistory=[]
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
	    print 'img: ' , img
            img=np.array(img)
	    slope_x=img[0,0]
            yint_x=img[0,1]
	    slope_y=img[0,2]
            yint_y=img[0,3]
            frame_number=img[0,4]
           # print 'x:: slop: ' , slope_x, ' yint: ' , yint_x,'x:: slop: ' , slope_y, ' yint: ' , yint_y
            print frame_number
	    print img
            yint_xHistory.append(yint_x)
            yint_yHistory.append(yint_y)
            slope_xHistory.append(slope_x)
            slope_yHistory.append(slope_y)
	    plt.plot(yint_xHistory)
	    plt.draw()
	   # plt.plot(slope_xHistory)
	    #plt.draw()
	   # plt.plot(yint_yHistory)
	    #plt.draw()
	    #plt.plot(slope_yHistory)
	    #plt.draw()
	    time.sleep(0.05)
        

    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
