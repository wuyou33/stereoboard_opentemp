import numpy as np
import cv2
import serial
import matplotlib.pyplot as plt
import stereoboard_tools
ser = serial.Serial('/dev/ttyUSB0',1000000,timeout=None)

frameNumber = 0
cv2.namedWindow('imgmatrixboard',cv2.WINDOW_NORMAL)

def draw_sonar_visualisation(matrix,height):
    plt.ion()
    r = matrix[1,:]
    r = (map(abs, map(int, r)))
    theta = np.arange(0,2*np.pi,(2*np.pi)/len(r))
    ax = plt.subplot(111, polar=True)
    ax.clear()
    colors = ['r', 'g', 'b', 'y', 'k']
    for i in range(0, height):
        r = matrix[i, :]
        r = (map(abs, map(int, r)))
        print 'R in loop: ', r , ' of ', matrix.shape
        print 'theta: ', len(theta), ' R: ', len(r)
        ax.plot(theta, r, color=colors[i], linewidth=3)
    ax.set_rmax(20.0)
    ax.grid(True)
    plt.draw()


def fill_matrix_multigaze_array(raw, width,height):
    img=np.zeros((height,width))
    place=0
    line=0
    # Fill the image arrays
    for i in range(0, width*height+height*6 + 4):
        if (raw[i] == 255) and (raw[i + 1] == 0) and (raw[i + 2] == 0):
            if (raw[i + 3] == 128):
                #  'startOfBuf: ', i
                # Start Of Line
                startOfBuf = i + 4
                endOfBuf = (i + 4 + width)
                lineBuffer = []
                for element in raw[startOfBuf:endOfBuf]:
                    lineBuffer.append(element)

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

    return img

currentBuffer=[]
while True:
    try:
        # Read the image
        currentBuffer, location = stereoboard_tools.readPartOfImage(ser, currentBuffer)

        if location > 0:
            oneImage = currentBuffer[0:location]
            currentBuffer=currentBuffer[location::]

            # Search the startbyte
            sync1, length,lineLength, lineCount=stereoboard_tools.determine_image_and_line_length(oneImage)
            # print 'sync ' , sync1, ' ', length, ' ', lineLength, ' ', lineCount

            if sync1<0:    # We did not find the startbit... try again
                continue


            # Initialise image
            img = np.zeros((lineLength,lineCount))

            line=0
            imgMatrix = np.zeros((lineCount, lineLength))
            imgMatrix = fill_matrix_multigaze_array(oneImage,lineLength,lineCount)

            # draw_sonar_visualisation(imgMatrix)
            imgMatrix /= 45
            cv2.imshow('imgmatrixboard',imgMatrix)
            cv2.waitKey(1)
    except Exception as excepte:
        print 'error: ', excepte