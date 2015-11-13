Stereocam projects
==================

BOARD=calibrate
---------------

 - Send pixmuxed stereo image pair
 - use pythontools stereocam_calibrate.py to view and adjust
 - save the calibration values into a BOARD file


BOARD=explorer
--------------

 - Droplet: sends 9600 bps 'a' 'b' 'c' 'd' stages of droplet avoidance manoeuvre

BOARD=stereo_avoid_demo
-----------------------

 - Sends disparity value

BOARD=send_distance_matrix
--------------------------

 - Streams 9600 reduced size disparity image as a 11x11 matrix
 - use paparazzi stereocam.xml module to decode
