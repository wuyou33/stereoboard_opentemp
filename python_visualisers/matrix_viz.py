import numpy as np
import cv2
import serial
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyUSB0',3000000,timeout=None)
size_of_one_image=50
frameNumber = 0
BINS=4
MATRIX_WIDTH=4
MATRIX_HEIGHT=4

def draw_sonar_visualisation(matrix):
    plt.ion()
    r = matrix[1, :] * 30
    r = (map(abs, map(int, r)))
    RADIANTS = 0.8
    START=0.55
    theta = np.arange(START, np.pi * RADIANTS, (RADIANTS * np.pi) / len(r))
    ax = plt.subplot(111, polar=True)
    ax.clear()
    colors = ['r', 'g', 'b', 'y', 'k']
    for i in range(0, BINS):
        r = matrix[i, :] * 10
        r = (map(abs, map(int, r)))
        ax.plot(theta, r, color=colors[i], linewidth=3)
    ax.set_rmax(15.0)
    ax.grid(True)
    plt.draw()

def search_start_position(startPosition):
    sync=0
    # Search for the startposition
    for i in range(startPosition, size_of_one_image):
        if (raw[i] == 255) and (raw[i + 1] == 0) and (raw[i + 2] == 0):
            if (raw[i + 3] == 171):
                # End of Image
                sync = i
                break
    return sync


def fill_matrix_array(startSync):
    global i, startOfBuf, endOfBuf, lineBuffer, line
    # Fill the image arrays
    for i in range(startSync, size_of_one_matrix + startSync):
        if (raw[i] == 255) and (raw[i + 1] == 0) and (raw[i + 2] == 0):
            if (raw[i + 3] == 128):
                #  'startOfBuf: ', i
                # Start Of Line
                startOfBuf = i + 4
                endOfBuf = (i + 4 + MATRIX_WIDTH)
                lineBuffer = raw[startOfBuf:endOfBuf]
                print 'lineis now> ', line
                imgMatrix[line, :] = lineBuffer
                line += 1
            else:
                # print 'maybe end of image'
                #if (raw[i+3] == 218):
                # End Of Line
                #    print 'EOL'
                #else:
                if (raw[i + 3] == 171):
                    # End of Image
                    print 'END OF IMAGE'
                    # Go from values between 0-255 to intensities between 0.0-1.0
                    #  print 'IMG matrix: ', imgMatrix

while True:
    # Read the image
    raw = ser.read(size_of_one_image*2)    # Read two times the image size... this way we surely have an image
    raw = bytearray(raw)

    # Initialise image
    img = np.zeros((BINS,BINS))

    # Initialise the startposition in the buffer and the linenumber
    line =0

    # Search the startbyte
    sync1 = search_start_position(0)
    print 'sync 1 ' , sync1
    if sync1==0:    # We did not find the startbit... try again
        continue


    line=0
    imgMatrix = np.zeros((4, MATRIX_WIDTH))
    size_of_one_matrix=MATRIX_HEIGHT*MATRIX_WIDTH+4*MATRIX_HEIGHT+4*MATRIX_HEIGHT+4

    fill_matrix_array(sync1)
    print imgMatrix
    imgMatrix /= 45
    imgMatrix =imgMatrix[:,::-1]
    print imgMatrix
    cv2.namedWindow('imgmatrixboard',cv2.WINDOW_NORMAL)
    cv2.imshow('imgmatrixboard',imgMatrix)


    draw_sonar_visualisation(imgMatrix)
    cv2.waitKey(1)