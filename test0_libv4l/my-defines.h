#ifndef _NXT_DEFINES_
#define _NXT_DEFINES_

	#define VENDOR_ID 1684
	#define PRODUCT_ID 2

	#define OUT_A 0x00
	#define OUT_B 0x01
	#define OUT_C 0x02
	#define OUT_ABC 0xff

	#define MODE_ON 0x01
	#define MODE_BRAKE 0x02
	#define MODE_REGULATED 0x04

	#define REGMODE_IDLE 0x00
	#define REGMODE_SPEED 0x01
	#define REGMODE_SYNC 0x02

	#define RUNSTATE_IDLE 0x00
	#define RUNSTATE_RAMPUP 0x10
	#define RUNSTATE_RUN 0x20
	#define RUNSTATE_RAMPDOWN 0x40

	#define TACHO_NO_LIMIT 0x00
#endif

#ifndef _OPEN_DEFINES_
#define _OPEN_DEFINES_

	#define CV_BGR2GRAY 6
	#define CV_BGR2HSV 40
	#define CV_RGB2HSV 41
	#define CV_RGB2BGR 4
	#define CV_BGR2RGB CV_RGB2BGR
	#define CV_CAP_PROP_FRAME_WIDTH 3
	#define CV_CAP_PROP_FRAME_HEIGHT 4

#endif

#ifndef _MY_OPEN_DEFINES_
#define _MY_OPEN_DEFINES_

	#define MIN_RED 170
	#define MAX_RED 255

	#define MIN_GREEN 0
	#define MAX_GREEN 120

	#define MIN_BLUE 0
	#define MAX_BLUE 120

	#define MIN_HUE 0
	#define MAX_HUE 5

	#define MIN_VALUE 0
	#define MAX_VALUE 255

	#define MIN_SATURATION 150
	#define MAX_SATURATION 255

#endif

#ifndef _MY_V4L_DEFINES_
#define _MY_V4L_DEFINES_

	#define WIDTH 320
	#define HEIGHT 240
	#define NUM_OF_CAPS -1

	#define DEVICE_NAME "/dev/video0"

#endif

#ifndef _NXT_MOTOR_MOVE_DEFINES_
#define _NXT_MOTOR_MOVE_DEFINES_

	#define KP 3
	#define initialSpeed 75

	#define MAX_SPEED 100
	#define MIN_SPEED 60

#endif

#ifndef _MY_DEFINES_INCLUDED_
#define _MY_DEFINES_INCLUDED_

//#include <opencv2/opencv.hpp>
//#include <opencv/highgui.h>
//#include <opencv2/core/types_c.h>
//#include <opencv2/imgproc/types_c.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include "libv4l2.h"
#include <linux/v4l2-mediabus.h>

//#include "dlib.h"

#endif

