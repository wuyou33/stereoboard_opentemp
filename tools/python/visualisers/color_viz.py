#! /usr/bin/python

import cv2
import sys
import argparse
from os import path, getenv
import serial
import stereoboard_tools
import numpy as np

sys.path.append('/usr/local/lib/python2.7/site-packages')

class Viewer:
    mouse = dict()
    currentBuffer=[]
    scale = 6
    imgRGB = np.zeros((96*scale,128*scale,3), dtype="uint8")
    imgYUV = np.zeros((96,128,3), dtype="uint8")
    print_other = 0

    def __init__(self, other):
        # Create a named window and add a mouse callback
        cv2.namedWindow('image',cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('image', self.on_mouse)
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
                
                if endOfImagesFound > 0:
                  sync1, length, lineLength, lineCount = stereoboard_tools.determine_image_and_line_length(self.currentBuffer[startPosition:endPosition])

                  data = stereoboard_tools.fill_image_array(sync1, self.currentBuffer[startPosition:endPosition], lineLength, lineCount)
                  
                  self.currentBuffer = self.currentBuffer[endPosition::]
                  if (endPosition - startPosition >= 128):
                      self.imgYUV[:,   :,0] = data[:,1::2]
                      self.imgYUV[:,0::2,1] = data[:, ::4]
                      self.imgYUV[:,1::2,1] = data[:, ::4]
                      self.imgYUV[:, ::2,2] = data[:,2::4]
                      self.imgYUV[:,1::2,2] = data[:,2::4]

                      # Create a color image
                      self.imgRGB = cv2.cvtColor(self.imgYUV, cv2.COLOR_YUV2RGB)
                      # if (not '3.0.0'==cv2.__version__) and (not '3.0.0-dev'==cv2.__version__):
                      self.imgRGB = cv2.resize(self.imgRGB,(0,0),fx=self.scale,fy=self.scale ,interpolation=cv2.INTER_CUBIC)
                  elif self.print_other:
                    print(data)
                    
            except Exception as excep:
                stereoboard_tools.PrintException()
                print 'error! ' , excep
            
            # Run the computer vision function
            self.cv()

            # Process key input
            self.on_key(cv2.waitKey(1) & 0xFF)

    def cv(self):
        # Show the image in a window
        cv2.resizeWindow('image', len(self.imgRGB[0,:,0]), len(self.imgRGB[:,0,0]))
        cv2.imshow('image', self.imgRGB)

    def on_key(self, key):
        if key == ord('q'):
            self.running = False

    def on_mouse(self, event, y, x, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            x_ = x/self.scale
            y_ = y/self.scale
            print('x,y', x_, y_, self.imgYUV[x_,y_,0], self.imgYUV[x_,y_,1], self.imgYUV[x_,y_,2])

if __name__ == '__main__':
    import sys
    import os
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", type=str, default='/dev/ttyUSB0', help="The port name of the camera (/dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=1000000, help="The baud rate of the camera (1000000)")
    parser.add_argument("-s", "--save", type=int, default=0, help="Whether or not to save incoming images (0 or 1)")
    parser.add_argument("-o", "--other", type=int, default=0, help="Print other messages from camera (0 or 1)")

    args = parser.parse_args()
        
    ser = serial.Serial(args.port,args.baud,timeout=None)
    saveImages = args.save
      
    viewer = Viewer(args.other)
    viewer.run()
    viewer.cleanup()


