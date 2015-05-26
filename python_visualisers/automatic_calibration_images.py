import numpy as np
import cv2
import serial
import array
import sys
import math
import itertools

def changeCanny1(newValue):
    global  firstCannyArgument
    firstCannyArgument = newValue
def changeCanny2(newValue):
    global  secondCannyArgument
    secondCannyArgument = newValue
def changeOffsetLeft(newValue):
    global  DISPARITY_OFFSET_LEFT
    DISPARITY_OFFSET_LEFT = newValue-maxOffset/2

def changeOffsetRight(newValue):
    global  DISPARITY_OFFSET_RIGHT
    DISPARITY_OFFSET_RIGHT= newValue-maxOffset/2


W = 128
H=96
frameNumber = 0
DISPARITY_OFFSET=-2
firstCannyArgument=100
secondCannyArgument=200

timeoutTime = 1
maxOffset=16
cv2.namedWindow('img',cv2.WINDOW_NORMAL)
cv2.namedWindow('imgCanny',cv2.WINDOW_NORMAL)
#   cv2.namedWindow('leftimg',cv2.WINDOW_NORMAL)
#   cv2.namedWindow('rightimg',cv2.WINDOW_NORMAL)
#cv2.createTrackbar('firstArgument','imgCanny',0,255,changeCanny1)
#cv2.createTrackbar('secondArgument','imgCanny',0,255,changeCanny2)
cv2.createTrackbar('offsetLeft','imgCanny',maxOffset/2,maxOffset,changeOffsetLeft)
cv2.createTrackbar('offsetRight','imgCanny',maxOffset/2,maxOffset,changeOffsetRight)
saveImages=False
DISPARITY_OFFSET_LEFT=8
DISPARITY_OFFSET_RIGHT=8
DISPARITY_BORDER=W/2

for imageToLoad in range(0,88):
    try:
        folderName = 'images/'
        # imageToLoad = 88
        img = cv2.imread(folderName+'imageBoth'+str(imageToLoad)+'.png')
        # img= cv2.imread(folderName+'imageLeft'+str(imageToLoad)+'.png')
        leftImage= cv2.imread(folderName+'imageLeft'+str(imageToLoad)+'.png')
        rightImage = cv2.imread(folderName+'imageRight'+str(imageToLoad)+'.png')
        scores = []
        locations = []
        for offLeft, offRight, DISPARITY_BORDER in itertools.product(range(maxOffset), range(maxOffset), range(0,W*2,8)):
            cv2.setTrackbarPos('offsetLeft', 'imgCanny',offLeft)
            cv2.setTrackbarPos('offsetRight', 'imgCanny',offRight)


            #changeOffset(setOffset)
            # print leftImage.shape
            # print 'before: ', img.shape
            img = np.zeros((leftImage.shape[0]+maxOffset,leftImage.shape[1]*2,3),np.uint8)
            sameimg = np.zeros((leftImage.shape[0]+maxOffset,leftImage.shape[1]*2,3),np.uint8)
            # print 'new: ', img.shape
            offset = DISPARITY_OFFSET_LEFT
            for line in range(0,leftImage.shape[1]):
                if line == DISPARITY_BORDER:
                    offset = DISPARITY_OFFSET_RIGHT

                if offset >=0:
                    img[offset:96+offset,line*2,:] = leftImage[:,line,:]
                    img[0:96,line*2+1,:] = rightImage[:,line,:]
                else:
                    img[0:96,line*2,:] = leftImage[:,line,:]
                    img[abs(offset):96+abs(offset),line*2+1,:] = rightImage[:,line,:]

            highCut = max(DISPARITY_OFFSET_LEFT,DISPARITY_OFFSET_RIGHT)
            lowCut = H-highCut

            newImg = img[0:lowCut,:,:]
            img = newImg.copy()
            sameimg = np.zeros(img.shape,np.uint8)
            for line in range(0,img.shape[1]-1):
                sameimg[:,line,:] = img[:,line,:]==img[:,line+1,:] #* img[:,line,:] !=[0,0,0]
            sameimg*=255
            locations.append((offLeft,offRight, DISPARITY_BORDER))
            scores.append(int(np.sum(sameimg)/10000))
          #  print 'sum: ', np.sum(sameimg)
           # cv2.imshow('img',img)
          #  cv2.imshow('imgCanny',sameimg)


            #key=cv2.waitKey(timeoutTime)
        maxValue = max(scores)
        place=[i for i, j in enumerate(scores) if j == maxValue][0]

        DISPARITY_OFFSET_LEFT,DISPARITY_OFFSET_RIGHT, DISPARITY_BORDER = locations[place]
       # print 'scores: ', scores
      #  print 'place: ',place, 'values: ', DISPARITY_OFFSET_LEFT , '  ', DISPARITY_OFFSET_RIGHT
        cv2.setTrackbarPos('offsetLeft', 'imgCanny',DISPARITY_OFFSET_LEFT)
        cv2.setTrackbarPos('offsetRight', 'imgCanny',DISPARITY_OFFSET_RIGHT)
        img = np.zeros((leftImage.shape[0]+maxOffset,leftImage.shape[1]*2,3),np.uint8)
        sameimg = np.zeros((leftImage.shape[0]+maxOffset,leftImage.shape[1]*2,3),np.uint8)
        # print 'new: ', img.shape
        offset = DISPARITY_OFFSET_LEFT
        for line in range(0,leftImage.shape[1]):
            if line == DISPARITY_BORDER:
                offset = DISPARITY_OFFSET_RIGHT
            # img[line,1:2*DISPARITY_BORDER:2]=leftLine[0:DISPARITY_BORDER]
            # img[line+abs(DISPARITY_OFFSET_LEFT),0:2*halfWay:2]=rightLine[0:DISPARITY_BORDER]
            # img[line,2*DISPARITY_BORDER+1::2]=leftLine[DISPARITY_BORDER::]
            # img[line+abs(DISPARITY_OFFSET_RIGHT),2*DISPARITY_BORDER+0::2]=rightLine[DISPARITY_BORDER::]
            # #print 'now doing line: ', leftImage[:,line,:]

            if offset >=0:
                img[offset:96+offset,line*2,:] = leftImage[:,line,:]
                img[0:96,line*2+1,:] = rightImage[:,line,:]
            else:
                img[0:96,line*2,:] = leftImage[:,line,:]
                img[abs(offset):96+abs(offset),line*2+1,:] = rightImage[:,line,:]
        #img/=255

        highCut = max(DISPARITY_OFFSET_LEFT,DISPARITY_OFFSET_RIGHT)
        lowCut = H-highCut
        # # lowCut = H+maxOffset-highCut
        newImg = img[0:lowCut,:,:]
        img = newImg.copy()
        sameimg = np.zeros(img.shape,np.uint8)
        for line in range(0,img.shape[1]-1):
            sameimg[:,line,:] = img[:,line,:]==img[:,line+1,:] #* img[:,line,:] !=[0,0,0]
        sameimg*=255
        locations.append((DISPARITY_OFFSET_LEFT,DISPARITY_OFFSET_RIGHT))
        scores.append(int(np.sum(sameimg)/10000))
        #print 'sum: ', np.sum(sameimg)


        print '#define DISPARITY_OFFSET_LEFT ', DISPARITY_OFFSET_LEFT
        print '#define DISPARITY_OFFSET_RIGHT ', DISPARITY_OFFSET_RIGHT
        print '#define DISPARITY_BORDER ', DISPARITY_BORDER

        cv2.imshow('img',img)
        cv2.imshow('imgCanny',sameimg)
        key=cv2.waitKey(5000)


    except Exception as exception:
        print 'error! ', exception
