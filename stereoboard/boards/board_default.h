#define BOARD_NUMBER	1

//#define CAMERA_CPLD_STEREO camera_cpld_stereo_left
//#define CAMERA_CPLD_STEREO camera_cpld_stereo_right
#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux
//#define CAMERA_CPLD_STEREO camera_cpld_stereo_linemux
#define DISPARITY_OFFSET_LEFT  0
#define DISPARITY_OFFSET_RIGHT  0
#define DISPARITY_BORDER  96

#define CAPTURE_MODE_SNAPSHOT 0   // No-snapshot, but continuous: We want fast frames without any delay, and it does not matter if the lower half of the buffer already contains a new image
