Python Visualisers
====================

We are currently in the process of pimping the visualisers, the following files have been updated thus far:
color_viz.py
stereocam_viz.py

A help output has been added to these files to give information on how the functions can be used. An example output is:
usage: stereocam_viz.py [-h] [-p PORT] [-b BAUD] [-o] [-s]

This will split and display the left and right images from the stereo camera.
Press q to quit and s to save a frame.

-h, --help            show this help message and exit
-p PORT, --port PORT  The port name of the camera (/dev/ttyUSB0)
-b BAUD, --baud BAUD  The baud rate of the camera (921600)
-o, --other           Print other messages from camera
-s                    Save all incoming images

As you can see, functionality such as selecting the baud rate and port can be selected as a command line input. Also, for the image visualisers, the option to save all images has been added.

Other files will be updated with this extended functionality in the future. We also hope to unify some functionality accross files to avoid to much copy pasting and reduce the maintainance time for functions.

