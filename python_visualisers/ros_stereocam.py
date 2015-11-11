#!/usr/bin/env python

#roslib.load_manifest('infiniumproject')
import roslib; roslib.load_manifest('infiniumproject')
import numpy as np
import cv2
import serial
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



def talker():
    pub = rospy.Publisher('chatter', String)
    leftCam = rospy.Publisher('stereocamera/left', Image)

    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    ser = serial.Serial('/dev/ttyUSB0',3000000)
    size_of_one_image=25348 # 128*96*2+4*96+4*96+4
    W = 128
    H=96
    bridge = CvBridge()

    DISPARITY_OFFSET=1
    while not rospy.is_shutdown():
        print 'now doing something'
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()

        # Read two times the image size... this way we surely have an image
        raw = ser.read(size_of_one_image*2)
        raw = bytearray(raw)

        # Initialise images
        img = np.zeros((H+abs(DISPARITY_OFFSET*3),W*2))
        leftImage=np.zeros((H,W))
        rightImage=np.zeros((H,W))

        # Initialise the startposition in the buffer and the linenumber
        sync=0
        line =0

        # Search for the startposition
        for i in range(1,size_of_one_image):
            if (raw[i] == 255) and (raw[i+1] == 0) and (raw[i+2] == 0):
                if (raw[i+3] == 171):
                    # End of Image
                    sync=i
                    break

        print 'sync is now: ', sync, ' length buffer; ', len(raw)
        if sync==0:
            # We did not find the startbit... try again
            continue

        # Fill the image arrays
        for i in range(sync,size_of_one_image+sync):
            if (raw[i] == 255) and (raw[i+1] == 0) and (raw[i+2] == 0):
                if (raw[i+3] == 128):
                    #print 'startOfBuf: ', i
                    # Start Of Line
                    startOfBuf = i+4
                    endOfBuf = (i+4+128+128)
                    lineBuffer = raw[startOfBuf:endOfBuf]
                    rightLine = lineBuffer[::2]
                    leftLine = lineBuffer[1:][::2]
                    #img[line,:]=lineBuffer
                    img[line+DISPARITY_OFFSET+abs(DISPARITY_OFFSET),::2]=leftLine
                    img[line+abs(DISPARITY_OFFSET),1::2]=rightLine
                    leftImage[line,:]=leftLine
                    rightImage[line,:]=rightLine
                    line+=1
                else:
                #print 'maybe end of image'
                #if (raw[i+3] == 218):
                # End Of Line
                #    print 'EOL'
                #else:
                    if (raw[i+3] == 171):
                        # End of Image
                        print 'END OF IMAGE'
            # Go from values between 0-255 to intensities between 0.0-1.0
        img /= 255
        leftImage /= 255
        rightImage /=255


        cv2.namedWindow('img',cv2.WINDOW_NORMAL)
        cv2.namedWindow('leftimg',cv2.WINDOW_NORMAL)
        cv2.namedWindow('rightimg',cv2.WINDOW_NORMAL)
        cv2.imshow('img',img)
        cv2.imshow('leftimg',leftImage)
        cv2.imshow('rightimg',rightImage)
        leftCam.publish(bridge.cv2_to_imgmsg(leftImage, "bgr8"))
        key=cv2.waitKey(1)



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

