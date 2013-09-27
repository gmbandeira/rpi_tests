#ifndef _NXT_DEFINES_
#define _NXT_DEFINES_

	#define VENDOR_ID 1684
	#define PRODUCT_ID 2
	#define MAC_ADDR 001653181487

	#ifndef _WIN32
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

	#define MIN_VALUE_RED 130
	#define MAX_VALUE_RED 256

	#define MIN_HUE_RED 50
	#define MAX_HUE_RED 256

	#define MIN_SATURATION_RED 150
	#define MAX_SATURATION_RED 256


	#define MIN_VALUE_BLUE 80
	#define MAX_VALUE_BLUE 140

	#define MIN_HUE_BLUE 0
	#define MAX_HUE_BLUE 256

	#define MIN_SATURATION_BLUE 100
	#define MAX_SATURATION_BLUE 256

#endif

#ifndef _MY_V4L_DEFINES_
#define _MY_V4L_DEFINES_

	#define WIDTH 160
	#define HEIGHT 120
	#define NUM_OF_CAPS -1

	#ifdef _WIN32
		#define DEVICE_NAME 0
	#else	
		#define DEVICE_NAME "/dev/video0"
	
	#endif

#endif

#ifndef _NXT_MOTOR_MOVE_DEFINES_
#define _NXT_MOTOR_MOVE_DEFINES_

	#define KP 3
	#define initialSpeed 30

	#define MAX_SPEED 100
	#define MIN_SPEED 20

#endif
