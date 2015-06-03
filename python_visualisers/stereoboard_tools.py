import numpy as np
import sys
import linecache


def PrintException():
    exc_type, exc_obj, tb = sys.exc_info()
    f = tb.tb_frame
    lineno = tb.tb_lineno
    filename = f.f_code.co_filename
    linecache.checkcache(filename)
    line = linecache.getline(filename, lineno, f.f_globals)
    print 'EXCEPTION IN ({}, LINE {} "{}"): {}'.format(filename, lineno, line.strip(), exc_obj)

## Determines the start, length of one image, and the length of one line and the width and height
def determine_image_and_line_length(raw):
    startPosition=None
    lineLength=0
    startLine=0
    lineCount=0
    startCounting=False
    for i in range(0, len(raw)):
        if (raw[i] == 255) and (raw[i + 1] == 0) and (raw[i + 2] == 0):
            if (raw[i + 3] == 171 and startPosition != None):# End of Image
                print 'found image length: ' , i -startPosition
                return startPosition, (i - startPosition),lineLength, lineCount
            if raw[i + 3] == 175:# Start of image
                startPosition = i
                startLine = None
                startCounting=True
            if  raw[i + 3] == 128: # Start of line
                startLine = i
            if raw[i + 3] == 218 and startCounting: # End of line
                lineLength = i-startLine-4
                lineCount+=1
    return 0,0,0,0


 # Fill the image arrays
def fill_image_arrays(raw, startposition, size_of_one_image, width, heigth, disparity_offset_left,disparity_offset_right,disparity_border):
    try:
        line=0
        # Initialise images
        img = np.zeros((heigth+max(abs(disparity_offset_left),abs(disparity_offset_right))*3,width*2))
        leftImage=np.zeros((heigth,width))
        rightImage=np.zeros((heigth,width))


        for i in range(startposition,size_of_one_image+startposition):
            if (raw[i] == 255) and (raw[i+1] == 0) and (raw[i+2] == 0):
                if (raw[i+3] == 128):# Start Of Line
                    startOfBuf = i+4
                    endOfBuf = (i+4+128+128)
                    lineBuffer = raw[startOfBuf:endOfBuf]
                    rightLine = lineBuffer[::2]
                    leftLine = lineBuffer[1:][::2]

                    halfWay = disparity_border

                    # Line indicates the horizontal line
                    img[line,1:2*halfWay:2]=leftLine[0:halfWay]
                    img[line+disparity_offset_left,0:2*halfWay:2]=rightLine[0:halfWay]
                    img[line,2*halfWay+1::2]=leftLine[halfWay::]
                    img[line+disparity_offset_right,2*halfWay+0::2]=rightLine[halfWay::]
                    leftImage[line,:]=leftLine
                    rightImage[line,:]=rightLine
                    line+=1
                else:
                    if (raw[i+3] == 171):
                        # End of Image
                        print 'END OF IMAGE'

        return img, leftImage, rightImage
    except Exception as e:
        PrintException()

def saveImages(img,leftImage,rightImage,frameNumber,folderName):
    import scipy
    fileNameLeft = folderName+'/imageLeft'+str(frameNumber)+'.png'
    fileNameRight = folderName+'/imageRight'+str(frameNumber)+'.png'
    fileNameBoth = folderName+'/imageBoth'+str(frameNumber)+'.png'
    scipy.misc.imsave(fileNameBoth, img)
    scipy.misc.imsave(fileNameLeft, leftImage)
    scipy.misc.imsave(fileNameRight, rightImage)