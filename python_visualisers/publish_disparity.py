#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
import csv
import time
from cv_bridge import CvBridge, CvBridgeError
def talker():
    ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
    frameNumber = 0
    currentBuffer=[]

    pub = rospy.Publisher('chatter', String, queue_size=10)
    pubImage = rospy.Publisher('chatterImage', Image)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    bridge = CvBridge()

    while not rospy.is_shutdown():
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
            # cv2.imshow("test",img)
            # cv2.waitKey(1)
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            print img
            print img.dtype
            imgc = np.array(img,dtype=np.uint8)
            print imgc
            print imgc.dtype
            pubImage.publish(bridge.cv2_to_imgmsg(imgc,"mono8"))
            rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep




    
