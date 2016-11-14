#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import yaml
import numpy as np
import matplotlib.pyplot as plt
import sensor_msgs
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from stereo_msgs.msg import DisparityImage
import csv
import time
from cv_bridge import CvBridge, CvBridgeError
import tf

def parse_yaml(filename):
  stream = file(filename, 'r')
  calib_data = yaml.load(stream)
  cam_info = sensor_msgs.msg.CameraInfo()
  cam_info.width = calib_data['image_width']
  cam_info.height = calib_data['image_height']
  cam_info.K = calib_data['camera_matrix']['data']
  cam_info.D = calib_data['distortion_coefficients']['data']
  cam_info.R = calib_data['rectification_matrix']['data']
  cam_info.P = calib_data['projection_matrix']['data']
  cam_info.distortion_model = calib_data['distortion_model']
  return cam_info

def talker():
    W = 128
    H=96
   
    DISPARITY_OFFSET_LEFT=0
    DISPARITY_OFFSET_RIGHT=0
    DISPARITY_BORDER=W/2
    previousLeftImage=None
    BAUDRATE=1000000
    ser = serial.Serial('/dev/ttyUSB0',BAUDRATE)
    size_of_one_image=25348 # 128*96*2+4*96+4*96+4
    ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
    frameNumber = 0
    currentBuffer=[]
    pubImageLeft = rospy.Publisher('left/image_raw', Image, queue_size=10)
    pubImageRight = rospy.Publisher('right/image_raw', Image, queue_size=10)
    pubImageFloatingPoint = rospy.Publisher('left/image_disparityfloat', Image, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    bridge = CvBridge()

    info = parse_yaml("/home/roland/catkin_ws/src/beginner_tutorials/scripts/cal.yml")
    print 'Read the following info ', info
    pubCameraInfoLeft = rospy.Publisher('left/camera_info', CameraInfo, queue_size=10)
    pubCameraInfoRight = rospy.Publisher('right/camera_info', CameraInfo, queue_size=10)

    br = tf.TransformBroadcaster()
   
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
 #           print 'length: ', length, ' count: ', lineCount, ' lineLength: ', lineLength, ' end of images found: ', endOfImagesFound
            if sync1<0:    # We did not find the startbit... try again
                continue
           
            # Fill the image arrays
            img, leftImage, rightImage = stereoboard_tools.fill_image_arrays(
                oneImage, sync1,size_of_one_image, W, H, DISPARITY_OFFSET_LEFT,DISPARITY_OFFSET_RIGHT,DISPARITY_BORDER)

            # Go from values between 0-255 to intensities between 0.0-1.0
            imgl = np.array(leftImage,dtype=np.uint8)
            imgr = np.array(rightImage,dtype=np.uint8)
            pubImageLeft.publish(bridge.cv2_to_imgmsg(imgl,"mono8"))
            pubImageRight.publish(bridge.cv2_to_imgmsg(imgr,"mono8"))

	    imgc2 = np.array(img,dtype=np.float32)
	    pubImageFloatingPoint.publish(bridge.cv2_to_imgmsg(imgc2,"32FC1"))
	    br.sendTransform((0,0, 0), tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(), "hello","world")
            pubCameraInfoLeft.publish(info)
            pubCameraInfoRight.publish(info)
            rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep



    
