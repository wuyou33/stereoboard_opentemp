#!/usr/bin/env python
import numpy as np
import sys

sys.path.append('/usr/local/lib/python2.7/site-packages')

import cv2
import serial
import stereoboard_tools
import array
import sys
import os
import socket
UDP_IP = ""
UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
print "started something"

img = np.zeros((96,256))

while True:
    data, addr = sock.recvfrom(512) # buffer size is 1024 bytes
    # print "received message:", data
    raw = bytearray(data)
    currentBuffer = []
    # Add all bytes to our total collection of bytes

    for byte in raw:
        currentBuffer.append(int(byte))
        # print 'read: ', int(byte)
    if currentBuffer[0] < img.shape[0] and len(currentBuffer) == img.shape[1]:
        # print 'read: ', currentBuffer
        img[currentBuffer[0],:]=currentBuffer
        img[currentBuffer[0],:]/=256
        img2 = cv2.resize(img,(0,0),fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
        cv2.imshow('img',img2)
        cv2.waitKey(1)
    else:
        print 'shape: ', len(currentBuffer), "ssfsf",img.shape[1]
        print 'sfsf;,',img.shape[0]
        print 'true?  ',currentBuffer[0] < img.shape[0]
        print 'second: ' , len(currentBuffer) == img.shape[1]
        print currentBuffer