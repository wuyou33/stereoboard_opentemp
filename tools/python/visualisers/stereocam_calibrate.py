#! /usr/bin/python

import cv2
import sys
import argparse
from os import path, getenv
import serial
import stereoboard_tools
import numpy as np
import math
from time import gmtime, strftime

sys.path.append('/usr/local/lib/python2.7/site-packages')

class Viewer:
    mouse = dict()
    currentBuffer=[]
    scale = 2
    interlaced = []
    diff = []
    max_rotation = 20
    max_offset = 20
    rotation = 0
    rotationL = np.array([[1.,0,0],[0,1.,0]])
    rotationR = np.array([[1.,0,0],[0,1.,0]])
    x_offset = 0
    y_offset = 0
    w = 0
    h = 0

    def update_rotation(self,val):
      self.rotation = (val-self.max_rotation)/4.
      self.rotationL = cv2.getRotationMatrix2D((self.w/2,self.h/2),self.rotation,1)
      self.rotationR = cv2.getRotationMatrix2D((self.w/2,self.h/2),-self.rotation,1)
      
    def update_x_offset(self, val):
      self.x_offset = val - self.max_offset
      
    def update_y_offset(self, val):
      self.y_offset = val - self.max_offset
          
    def __init__(self):
        # Create a named window and add a mouse callback
        cv2.namedWindow('interlaced images',cv2.WINDOW_NORMAL)
        cv2.createTrackbar('rotation', 'interlaced images', self.max_rotation, 2*self.max_rotation+1, self.update_rotation)
        cv2.createTrackbar('x_offset', 'interlaced images', self.max_offset, 2*self.max_offset+1, self.update_x_offset)
        cv2.createTrackbar('y_offset', 'interlaced images', self.max_offset, 2*self.max_offset+1, self.update_y_offset)
        
        cv2.namedWindow('image diff',cv2.WINDOW_NORMAL)
        
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
                
                #print endPosition - startPosition
                
                if endOfImagesFound > 0:
                  sync1, length, lineLength, lineCount = stereoboard_tools.determine_image_and_line_length(self.currentBuffer[startPosition:endPosition])

                  data = stereoboard_tools.fill_image_array(sync1, self.currentBuffer[startPosition:endPosition], lineLength, lineCount)
                  
                  self.currentBuffer = self.currentBuffer[endPosition::]
                  if (endPosition - startPosition >= 128):
                    leftImg = np.concatenate((data[:,2::2], data[:,0:1]), axis=1)
                    rightImg = data[:,1::2]
                    
                    self.h,self.w = leftImg.shape
                    
                    if type(self.rotationL) == np.ndarray:
                      leftImg = cv2.warpAffine(leftImg, self.rotationL,(leftImg.shape[1],leftImg.shape[0]))
                      rightImg = cv2.warpAffine(rightImg, self.rotationR,(rightImg.shape[1],rightImg.shape[0]))
                    
                    # first perform veritcal and horizontal shifts
                    if (self.x_offset > 0):
                      rightImg = np.concatenate((np.zeros((self.h, self.x_offset), dtype="uint8"),rightImg), axis=1)
                      leftImg = np.concatenate((leftImg, np.zeros((self.h, self.x_offset), dtype="uint8")), axis=1)
                    else:
                      leftImg = np.concatenate((np.zeros((self.h, -self.x_offset), dtype="uint8"),leftImg), axis=1)
                      rightImg = np.concatenate((rightImg, np.zeros((self.h, -self.x_offset), dtype="uint8")), axis=1)
                    if (self.y_offset > 0):
                      leftImg = np.concatenate((leftImg, np.zeros((self.y_offset,leftImg.shape[1]), dtype="uint8")), axis=0)
                      rightImg = np.concatenate((np.zeros((self.y_offset,rightImg.shape[1]), dtype="uint8"),rightImg), axis=0)
                    else:
                      leftImg = np.concatenate((np.zeros((-self.y_offset,leftImg.shape[1]), dtype="uint8"), leftImg), axis=0)
                      rightImg = np.concatenate((rightImg, np.zeros((-self.y_offset,rightImg.shape[1]), dtype="uint8")), axis=0)
                      
                    if type(self.interlaced) != np.ndarray or self.interlaced.size != (2*leftImg.size) :
                      self.interlaced = np.zeros((2*(leftImg.shape[0]), leftImg.shape[1]), dtype="uint8")
                    
                    self.interlaced[::2,:] = leftImg
                    self.interlaced[1::2,:] = rightImg
                    
                    self.diff = np.absolute(np.subtract(leftImg.astype(np.int16), rightImg.astype(np.int16)))
                    self.diff = self.diff.astype(np.uint8)
                    #16, 10 
                    
                    #cv2.resize(np.concatenate((data[:,2::2], data[:,0:1]), axis=1),(0,0),fx=self.scale,fy=self.scale ,interpolation=cv2.INTER_CUBIC)

                    # Run the computer vision function
                    self.cv()
                  elif printOther:
                    print(data)
                    
            except Exception as excep:
                stereoboard_tools.PrintException()
                print 'error! ' , excep
            
            # Process key input
            self.on_key(cv2.waitKey(1) & 0xFF)

    def cv(self):
        # Show the image in a window
        if type(self.interlaced) == np.ndarray:
          #cv2.resizeWindow('interlaced images', len(self.interlaced[0,:]), len(self.interlaced[:,0]))
          cv2.imshow('interlaced images', self.interlaced)
          #cv2.resizeWindow('image diff', len(self.diff[0,:]), len(self.diff[:,0]))
          cv2.imshow('image diff', self.diff)

    def on_key(self, key):
        if key == ord('q'):
            self.running = False

    def on_mouse(self, event, y, x, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            x_ = x/self.scale
            y_ = y/self.scale
            print('x,y', x_, y_, self.imgYUV[x_,y_,0], self.imgYUV[x_,y_,1], self.imgYUV[x_,y_,2])
            
    def print_output(self):
      # only rotate right image, gives better results with the integer rotation
      self.rotationL = cv2.getRotationMatrix2D((self.w/2,self.h/2),0,1)
      self.rotationR = cv2.getRotationMatrix2D((self.w/2,self.h/2),-2*self.rotation,1)
      rotationLt = self.rotationL[:,:2].T
      rotationRt = self.rotationR[:,:2].T
      width_reduction = int(math.ceil(max(self.rotationL[0,2],self.rotationR[0,2])))
      height_reduction = int(math.ceil(max(self.rotationL[1,2],self.rotationR[1,2])))
      
      roiL = np.array([width_reduction + max(self.x_offset,0),
                       height_reduction + max(self.y_offset,0),
                       self.w - width_reduction - max(-self.x_offset,0),
                       self.h - height_reduction - max(-self.y_offset,0)])
      roiR = np.array([width_reduction + max(-self.x_offset,0),
                       height_reduction + max(-self.y_offset,0),
                       self.w - width_reduction - max(self.x_offset,0),
                       self.h - height_reduction - max(self.y_offset,0)])
                   
      # width and height must be odd
      width = roiL[2] - roiL[0]
      height = roiL[3] - roiL[1]
      
      x_offsetL = roiL[0] - width_reduction
      y_offsetL = roiL[1] - height_reduction
      x_offsetR = roiR[0] - width_reduction
      y_offsetR = roiR[1] - height_reduction
      
      # generate zero centered indicies
      index_xL = range(roiL[0]-(self.w-1)/2,roiL[2]-(self.w-1)/2)
      index_yL = range(roiL[1]-(self.h-1)/2,roiL[3]-(self.h-1)/2)
      index_xR = range(roiR[0]-(self.w-1)/2,roiR[2]-(self.w-1)/2)
      index_yR = range(roiR[1]-(self.h-1)/2,roiR[3]-(self.h-1)/2)
      
      f = open('calibrations/calibration{0}'.format(strftime("%Y%m%d%H:%M:%S", gmtime())), 'w')
      f.write('static const uint16_t roi_left[4] = {{{0}, {1}, {2}, {3}}};\n'.format(roiL[0], roiL[1], roiL[2], roiL[3]))
      f.write('static const uint16_t roi_right[4] = {{{0}, {1}, {2}, {3}}};\n'.format(roiR[0], roiR[1], roiR[2], roiR[3]))
      f.write('static const uint16_t cal_width = {0};\n'.format(width))
      f.write('static const uint16_t cal_height = {0};\n'.format(height))
      f.write('static const uint32_t cal_size = {0};\n'.format(width*height))
      f.write('static const uint32_t calL[{0}] = {{\n'.format(width*height))
      # The left and right rotations are purposfully reversed below
      for y in index_yL:
        for x in index_xL:
          index = round(rotationLt[1,0]*x + rotationLt[1,1]*y + (self.h-1)/2.) * self.w + rotationLt[0,0]*x + rotationLt[0,1]*y + (self.w-1)/2.
          f.write(str(2*int(round(index))))
          f.write(',')
        f.write('\n')
      f.write('};\n')
        
      f.write('static const uint32_t calR[{0}] = {{\n'.format(width*height))
      for y in index_yR:
        for x in index_xR:
          index = round(rotationRt[1,0]*x + rotationRt[1,1]*y + (self.h-1)/2.) * self.w + rotationRt[0,0]*x + rotationRt[0,1]*y + (self.w-1)/2.
          f.write(str(2*int(round(index))+1))
          f.write(',')
        f.write('\n')
      f.write('};\n')
      
if __name__ == '__main__':
    import sys
    import os
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", type=str, default='/dev/ttyUSB0', help="The port name of the camera (/dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=921600, help="The baud rate of the camera (921600)")
    parser.add_argument("-s", action="store_true", help="Whether or not to save incoming images (0 or 1)")
    parser.add_argument("-o", "--other", type=int, default=0, help="Print other messages from camera (0 or 1)")

    args = parser.parse_args()
        
    ser = serial.Serial(args.port,args.baud,timeout=None)
    saveImages = args.s
    printOther = args.other
      
    viewer = Viewer()
    viewer.run()
    viewer.print_output()
    
