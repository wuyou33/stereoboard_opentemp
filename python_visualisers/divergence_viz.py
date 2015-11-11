import cv2
import serial
import stereoboard_tools
import numpy as np
import matplotlib.pyplot as plt
import time
import math
plt.axis([0,300, -0.5,0.5])
plt.ion()
plt.show()
   
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
height = 0
radperpx = 0
distance_pinhole = 0.03
angle_disp = 0
saveImages= False
currentBuffer=[]
FOV_x=57.4
FOV_y=45
maxExpectedInImage=250 # Each pixel is divided by this value to go to grayscale values for the cv2 image
cv2.namedWindow('img',cv2.WINDOW_NORMAL)
velocity_x=0
velocity_y=0
velocity_xHistory=[]
velocity_yHistory=[]
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
	    slope_x=(img[0,0]-100)/1000
            yint_x=(img[0,1]-100)/100
	    slope_y=(img[0,2]-100)/1000
            yint_y=(img[0,3]-100)/100
            height=img[0,4]
            
            radperpx=(FOV_x*math.pi/180)/128
            angle_disp=height/2*radperpx
            height_meters=distance_pinhole/math.tan(angle_disp);
            
            radperpx=(FOV_x*math.pi/180)/128
            angle_disp=yint_x*radperpx
            velocity_x=height_meters*math.tan(angle_disp)*25

            radperpx=(FOV_y*math.pi/180)/96
            angle_disp=yint_y*radperpx
            velocity_y=height_meters*math.tan(angle_disp)*25
            print velocity_x
            print velocity_y
            print height_meters


            velocity_xHistory.append(velocity_x)
            velocity_yHistory.append(velocity_y)
	    plt.plot(velocity_xHistory)
	    plt.draw()
            plt.plot(velocity_yHistory)
	    plt.draw()

	    time.sleep(0.05)
        

    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
