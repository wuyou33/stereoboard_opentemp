#!/usr/bin/env python
import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
import cv2
import serial
import stereoboard_tools
import Tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
import csv
import time
import numpy as np
from sklearn.cluster import MeanShift, estimate_bandwidth
from sklearn.datasets.samples_generator import make_blobs
import matplotlib.pyplot as plt
from itertools import cycle

ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)
frameNumber = 0
saveImages= False
treshold=0.3
max_time = 500;

currentBuffer=[]
print cv2.__version__
if '3.0.0'==cv2.__version__:
    cv2.namedWindow('img',cv2.WINDOW_NORMAL)
if '3.0.0-dev'==cv2.__version__:
    cv2.namedWindow('img',cv2.WINDOW_NORMAL)


fileToWrite=file("data.csv",'w')
dataWriter=csv.writer(fileToWrite)

plt.figure(1)
plt.clf()
plt.ion()
plt.show()
r,h,c,w = 20,20,30,30  # simply hardcoded the values
track_window = (c,r,w,h)

# main loop:
while True:
    try:
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

            if saveImages:
                import scipy
                fileNameBoth = 'image'+str(frameNumber)+'.png'
                scipy.misc.imsave(fileNameBoth, img)
            totalData=[frameNumber,time.time()]
	    print img
            img /= 20
            img /= 6
           


            # Create a color image
            # img=stereoboard_tools.createRedBlueImage(img,lineCount,lineLength)
            # img = img[::2]
            # if (not '3.0.0'==cv2.__version__) and (not '3.0.0-dev'==cv2.__version__):
            #     print 'resizing stuff!'

	    	# print 'test hier'
            #%% Generate sample data
            # centers = [[1, 1], [-.75, -1], [1, -1], [-3, 2]]
            # X, _ = make_blobs(n_samples=10000, centers=centers, cluster_std=0.6)
            # X = []
            #
            #
            # print dir(img)
            # print img.shape
            # print img.shape[0]
            # print img.shape[1]
            # samplesInStereo=0
            # for x in range(img.shape[0]):
            #     for y in range(img.shape[1]):
            #         if img[x,y]>0:
            #             # print img[x,y]
            #             X.append([x,y])
            #             samplesInStereo+=1
            # # X = np.concatenate(X)
            # X = np.array(X)
            # # print X
            # #%% Compute clustering with MeanShift
            #
            # # The bandwidth can be automatically estimated
            # bandwidth = estimate_bandwidth(X, quantile=.1,
            #                                n_samples=samplesInStereo/3)
            # ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
            # ms.fit(X)
            # labels = ms.labels_
            # cluster_centers = ms.cluster_centers_
            #
            # n_clusters_ = labels.max()+1
            #
            # #%% Plot result
            # plt.clf()
            # colors = cycle('bgrcmykbgrcmykbgrcmykbgrcmyk')
            # for k, col in zip(range(n_clusters_), colors):
            #     my_members = labels == k
            #     cluster_center = cluster_centers[k]
            #     plt.plot(X[my_members, 0], X[my_members, 1], col + '.')
            #     plt.plot(cluster_center[0], cluster_center[1],
            #              'o', markerfacecolor=col,
            #              markeredgecolor='k', markersize=14)
            # plt.title('Estimated number of clusters: %d' % n_clusters_)
            # plt.draw()
            # time.sleep(0.05)
            #




            term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
            ret, track_window = cv2.CamShift(img, track_window, term_crit)
            print "ret: " , ret
            print "Track window: ", track_window
            x,y,w,h = track_window
            pts = cv2.boxPoints(ret)
            pts = np.int0(pts)
            img = cv2.polylines(img,[pts],True, 255,2)
            img = cv2.resize(img,(0,0),fx=5,fy=5,interpolation=cv2.INTER_NEAREST)
            cv2.imshow('img',img)
            key=cv2.waitKey(1)
            if 'q' == chr(key & 255):
                break	
            if saveImages:
                import scipy
                fileNameBoth = 'imageBoth'+str(frameNumber)+'.png'
                scipy.misc.imsave(fileNameBoth, img)
                frameNumber+=1

    except Exception as excep:
        stereoboard_tools.PrintException()
        print 'error! ' , excep
