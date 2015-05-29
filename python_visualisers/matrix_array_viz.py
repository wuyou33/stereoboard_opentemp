import numpy as np
import cv2
import serial
import matplotlib.pyplot as plt
import stereoboard_tools
ser = serial.Serial('/dev/ttyUSB1',1000000,timeout=None)
size_of_one_image=200
size_of_one_matrix=size_of_one_image
frameNumber = 0
AMOUNT_OF_BOARDS=6
SINGLE_MATRIX_WIDTH=5
MATRIX_HEIGHT=5
MATRIX_WIDTH=AMOUNT_OF_BOARDS*SINGLE_MATRIX_WIDTH

def draw_sonar_visualisation(matrix):
    plt.ion()
    r = matrix[1,:]
    r = (map(abs, map(int, r)))
    theta = np.arange(0,2*np.pi,(2*np.pi)/len(r))
    ax = plt.subplot(111, polar=True)
    ax.clear()
    colors = ['r', 'g', 'b', 'y', 'k']
    for i in range(0, MATRIX_HEIGHT):
        r = matrix[i, :]
        r = (map(abs, map(int, r)))
        print 'R in loop: ', r , ' of ', matrix.shape
        print 'theta: ', len(theta), ' R: ', len(r)
        ax.plot(theta, r, color=colors[i], linewidth=3)
    ax.set_rmax(50.0)
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



def fill_matrix_multigaze_array(raw,startSync,size_of_one_matrix):
    img=np.zeros((MATRIX_HEIGHT,MATRIX_WIDTH))
    place=0
    line=0
    # Fill the image arrays
    for i in range(startSync+4, MATRIX_WIDTH*MATRIX_HEIGHT+MATRIX_HEIGHT*6 + 4+startSync):
        if (raw[i] == 255) and (raw[i + 1] == 0) and (raw[i + 2] == 0):
            if (raw[i + 3] == 128):
                #  'startOfBuf: ', i
                # Start Of Line
                startOfBuf = i + 4
                endOfBuf = (i + 4 + MATRIX_WIDTH)
                lineBuffer = []
                for element in raw[startOfBuf:endOfBuf]:
                    lineBuffer.append(element)
                print 'lineis now> ', line,  'buffer: ', lineBuffer
                img[line, :] = lineBuffer
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

#
#     for i in range(startSync+4, MATRIX_WIDTH*MATRIX_HEIGHT + 4+startSync,16):
#         singleMatrix = []
#         for j in range(0,16):
#             print  'appending: ', raw[i+j]
#             singleMatrix.append(raw[i+j])
#
#
#
# #        singleMatrix = raw[i:i+16]
#         print 'single matrix: ', singleMatrix
#         for j in range(0,4):
#             #print 'to replace: ' , place , '  ', newMatrix[place:place+4,j]
#             #print 'with : ' , singleMatrix[j*4:(j+1)*4]
#             newMatrix[place:place+4,j]=singleMatrix[j*4:(j+1)*4]
#         place+=4
    print 'found img: ' , img
    return img


while True:
    try:
        # Read the image
        raw = ser.read(size_of_one_image*2)    # Read two times the image size... this way we surely have an image
        raw = bytearray(raw)

        # Initialise image
        img = np.zeros((MATRIX_WIDTH,MATRIX_HEIGHT))

        # Initialise the startposition in the buffer and the linenumber
        line =0

        # Search the startbyte
        sync1 = stereoboard_tools.search_start_position(raw,0,size_of_one_matrix)


        print 'sync 1 ' , sync1
        if sync1==0:    # We did not find the startbit... try again
            continue

        for i in range(sync1,sync1+(MATRIX_WIDTH*MATRIX_HEIGHT)):
            print raw[i], ' '
        line=0
        imgMatrix = np.zeros((MATRIX_HEIGHT, MATRIX_WIDTH))
        size_of_one_matrix=MATRIX_HEIGHT*MATRIX_WIDTH
        # #
        imgMatrix = fill_matrix_multigaze_array(raw,sync1,size_of_one_image)
        print imgMatrix
        draw_sonar_visualisation(imgMatrix)
        imgMatrix /= 45
        #imgMatrix =imgMatrix[:,::-1]
        # print imgMatrix
        cv2.namedWindow('imgmatrixboard',cv2.WINDOW_NORMAL)
        cv2.imshow('imgmatrixboard',imgMatrix)
        #
        #

        cv2.waitKey(1)
    except Exception as excepte:
        print 'error: ', excepte