#! /usr/bin/python

import cv2
import sys
import argparse
from os import path, getenv
import serial
import stereoboard_tools
import numpy as np

sys.path.append('/usr/local/lib/python2.7/site-packages')

cv2.namedWindow('Right image',cv2.WINDOW_NORMAL)
cv2.namedWindow('Left image',cv2.WINDOW_NORMAL)

class Viewer:
    mouse = dict()
    currentBuffer=[]
    scale = 2
    leftImg = []
    rightImg = []
    print_other = 0
    image = 1
    
    def __init__(self, other):
        # Create a named window and add a mouse callback
        self.print_other = other

    def run(self):
        self.running = True

        # Start an 'infinite' loop
        while self.running:
            # Read a frame from the video capture
            try:
                # Read the image
                self.currentBuffer, location, endOfImagesFound = stereoboard_tools.readPartOfImage(ser, self.currentBuffer)
                startPosition = location[0]
                endPosition = location[1]
                
                # print endPosition - startPosition
                if endOfImagesFound > 0:
                  sync1, length, lineLength, lineCount = stereoboard_tools.determine_image_and_line_length(self.currentBuffer[startPosition:endPosition])

                  data = stereoboard_tools.fill_image_array(sync1, self.currentBuffer[startPosition:endPosition], lineLength, lineCount)
                  
                  self.currentBuffer = self.currentBuffer[endPosition::]
                  if (endPosition - startPosition >= 128):
                    # Fill the image arrays
                    # NB The left image is rotated one pixel due to pixmux image intensity latch
                    #self.leftImg = cv2.resize(np.concatenate((data[:,2::2], data[:,0:1]), axis=1),(0,0),fx=self.scale,fy=self.scale ,interpolation=cv2.INTER_CUBIC)
                    self.leftImg = cv2.resize(data[:,::2],(0,0),fx=self.scale,fy=self.scale ,interpolation=cv2.INTER_CUBIC)
                    self.rightImg = cv2.resize(data[:,1::2],(0,0),fx=self.scale,fy=self.scale ,interpolation=cv2.INTER_CUBIC)
                    
                    if saveImages:
                      cv2.imwrite('images/left{0}.png'.format(self.image),self.leftImg)
                      cv2.imwrite('images/right{0}.png'.format(self.image),self.rightImg)
                      self.image = self.image + 1
                    
                    # Run the computer vision function
                    self.cv()
                    
                  elif self.print_other:
                    print(data)
                    
            except Exception as excep:
                stereoboard_tools.PrintException()
                print 'error! ' , excep
            
            # Process key input
            self.on_key(cv2.waitKey(10))

    def cv(self):
        # Show the image in a window
        if type(self.leftImg) == np.ndarray:
          cv2.resizeWindow('Left image', len(self.leftImg[0,:]), len(self.leftImg[:,0]))
          cv2.imshow('Left image', self.leftImg)
          cv2.resizeWindow('Right image', len(self.rightImg[0,:]), len(self.rightImg[:,0]))
          cv2.imshow('Right image', self.rightImg)

    def on_key(self, key):
        if key == ord('q'):
            self.running = False
        elif key == ord('s') and ~saveImages and type(self.leftImg) == np.ndarray:
          cv2.imwrite('images/left{0}.png'.format(self.image),self.leftImg)
          cv2.imwrite('images/right{0}.png'.format(self.image),self.rightImg)
          self.image = self.image + 1

    def on_mouse(self, event, y, x, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            x_ = x/self.scale
            y_ = y/self.scale
            print('x,y', x_, y_, self.imgYUV[x_,y_,0], self.imgYUV[x_,y_,1], self.imgYUV[x_,y_,2])

if __name__ == '__main__':
    import sys
    import os
    
    parser = argparse.ArgumentParser(description='This will split and display the left and right images from the stereo camera. Press q to quit and s to save a frame.')
    parser.add_argument("-p", "--port", type=str, default='/dev/ttyUSB0', help="The port name of the camera (/dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=921600, help="The baud rate of the camera (921600)")
    parser.add_argument("-o", "--other", action='store_true', help="Print other messages from camera")
    parser.add_argument("-s", action='store_true', help="Save all incoming images")

    args = parser.parse_args()
        
    ser = serial.Serial(args.port,args.baud,timeout=None)
    saveImages = args.s
    
    viewer = Viewer(args.other)
    viewer.run()
    cv2.destroyAllWindows()

