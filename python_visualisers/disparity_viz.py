import numpy as np
import cv2
import serial
import matplotlib.pyplot as plt
import stereoboard_tools
ser = serial.Serial('/dev/ttyUSB0',3000000,timeout=None)
size_of_one_image=25348
W = 128
H=96
frameNumber = 0
saveImages= False
BINS=4


def draw_sonar_visualisation(matrix):
    global X, Y, graph, r, RADIANTS, theta, ax, colors, i
    X = range(0, 15)
    Y = range(0, 15)
    plt.ion()
    graph = plt.plot(X, Y)[0]
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


def fill_disparity_array(startSync):
    global i, startOfBuf, endOfBuf, line
    # Fill the image arrays
    for i in range(startSync + 4, (size_of_one_image / 2) + startSync, 136):
        if (raw[i] == 255) and (raw[i + 1] == 0) and (raw[i + 2] == 0):
            if (raw[i + 3] == 128):
                # print i
                # Start Of Line
                startOfBuf = i + 4
                endOfBuf = (i + 4 + 128)

                img[line, :] = raw[startOfBuf:endOfBuf]
                line += 1;
                ## START MATRIX
                # Search for the startposition


while True:
    # Read the image
    raw = ser.read(size_of_one_image*2)    # Read two times the image size... this way we surely have an image
    raw = bytearray(raw)

    # Initialise image
    img = np.zeros((H, W))

    # Initialise the startposition in the buffer and the linenumber
    sync=0
    line =0

    # Search the startbyte
    sync1 = stereoboard_tools.search_start_position(raw,0,size_of_one_image)
    print 'sync 1 ' , sync
    if sync1==0:    # We did not find the startbit... try again
        continue



    #Start with the distance matrix
    sync2 = stereoboard_tools.search_start_position(raw,sync+4,size_of_one_image)
    print 'sync 2 ', sync2
    if sync2==0:
        continue # We did not find the startbit... try again

    print 'difference', sync2-sync1
    if sync2-sync1 > 52:
        imageSync = sync1
        matrixSync = sync2
    else:
        imageSync = sync2
        matrixSync = sync1

    fill_disparity_array(imageSync)
    #
    # line=0
    # MATRIX_WIDTH=4
    # MATRIX_HEIGHT=4
    # imgMatrix = np.zeros((4, MATRIX_WIDTH))
    # size_of_one_matrix=MATRIX_HEIGHT*MATRIX_WIDTH+4*MATRIX_HEIGHT+4*MATRIX_HEIGHT+4
    #
    # fill_matrix_array(matrixSync)
    #
    # imgMatrix /= 45
    # imgMatrix =imgMatrix[:,::-1]
    # cv2.namedWindow('imgmatrixboard',cv2.WINDOW_NORMAL)
    # cv2.imshow('imgmatrixboard',imgMatrix)


    img =img[:,::-1]
    img /= 100
    cv2.namedWindow('img',cv2.WINDOW_NORMAL)
    cv2.imshow('img',img)



#    draw_sonar_visualisation(imgMatrix)
    key=cv2.waitKey(1)

    if saveImages:
        import scipy
        fileNameBoth = 'imageBoth'+str(frameNumber)+'.png'
        scipy.misc.imsave(fileNameBoth, img)
        frameNumber+=1

