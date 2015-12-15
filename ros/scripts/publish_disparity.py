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
  cam_info = CameraInfo()
  cam_info.width = calib_data['image_width']
  cam_info.height = calib_data['image_height']

  cam_info.K = calib_data['camera_matrix']['data']
  cam_info.D = calib_data['distortion_coefficients']['data']
  cam_info.R = calib_data['rectification_matrix']['data']
  cam_info.P = calib_data['projection_matrix']['data']
  cam_info.distortion_model = calib_data['distortion_model']
  return cam_info

def talker():
    ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
    frameNumber = 0
    currentBuffer=[]
    info = parse_yaml("/home/roland/catkin_ws/src/beginner_tutorials/scripts/cal.yml")

    pubImage = rospy.Publisher('imageStereoboard', Image, queue_size=10)
    pubImage16 = rospy.Publisher('imageStereoboardSixteen', Image, queue_size=10)
    pubImageFloatingPoint = rospy.Publisher('disparityImageFloat', Image, queue_size=10)
    pubImageDisp = rospy.Publisher('stereocamDisparityImage', DisparityImage, queue_size=10)
    pubCameraInfoLeft = rospy.Publisher('stereocamCameraLeft', CameraInfo, queue_size=10)
    pubCameraInfoRight = rospy.Publisher('stereocamCameraRight', CameraInfo, queue_size=10)

    pubTransform = tf.TransformBroadcaster()
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    bridge = CvBridge()
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
            img = stereoboard_tools.fill_image_array(sync1,oneImage, lineLength, lineCount)
            img=np.array(img)

            imgc = np.array(img,dtype=np.uint8)

            pubImage.publish(bridge.cv2_to_imgmsg(imgc,"mono8"))
            imgc16 = np.array(img,dtype=np.uint16)
            imgc16[imgc16==0]=1337
            imgc16=(118*60)/imgc16
            imgc16[imgc16==(118*60)/1337]=0
            toPublish = bridge.cv2_to_imgmsg(imgc16,"16UC1")
            toPublish.header.frame_id="my_frame"
            pubImage16.publish(toPublish)
            msgout = DisparityImage()

            imgc2 = np.array(img,dtype=np.float32)
            msgout.image=bridge.cv2_to_imgmsg(imgc2,"32FC1")
            pubImageFloatingPoint.publish(bridge.cv2_to_imgmsg(imgc2,"32FC1"))
            br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "hello",
                         "world")
	    pubTransform.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "my_frame",
                         "base_link")
            pubCameraInfoLeft.publish(info)
            pubCameraInfoRight.publish(info)
            msgout.max_disparity=500.0
            pubImageDisp.publish(msgout)
            rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep




    
