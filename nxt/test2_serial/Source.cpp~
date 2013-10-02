
// g++ Source.cpp `pkg-config opencv --libs --cflags` -lusb-1.0 -lv4l2 -I/usr/local/include -L/usr/local/lib -lwiringPi -O3

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/types_c.h>

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
	#include <libv4l2.h>
	#include <linux/v4l2-mediabus.h>
	#include <libusb-1.0/libusb.h>
	#include <wiringPi.h>

	#define CLEAR(x) memset(&(x), 0, sizeof(x))

	struct buffer
	{
		void   *start;
		size_t length;
	};

#include "my-defines.h"

#define OPENCV_ESSENTIAL
#define NXT_ESSENTIAL

void	loop			();
void	legoAct			(int error);
void	merge			(cv::Mat dest, cv::Mat img0, cv::Mat img1);
int		dir				(cv::Mat img, int proportional = 80, float precision = 1);
int		getDirection	(cv::Mat img, int proportional = 80, float precision = 1);
bool	isRed			(cv::Scalar dot);
bool	isBlue			(cv::Scalar dot);
void	getRed			(cv::Mat dest);
void	getBlue			(cv::Mat dest);
int		main			();
void	motorPID		(int error);
int		nxtClose		();

int		motorMove		(unsigned char motor, unsigned char speed = 50, unsigned char mode = 0x01, unsigned char regulation = 0x01, unsigned char turn = 0x90);
void	xioctl			(int fh, int request, void *arg);
void	saveCap			();
void	cvtToOpencv		();

unsigned char maxValue(cv::Mat img, int type = TYPE_GET_MAX_RED, int diff_red = 130, int diff_blue = 200);
void thresh(cv::Mat img, cv::Mat dest, int thresh_type = THRESH_TYPE_GET_RED, int thresh = 0, int diff_red = 30, int diff_blue = 10);


// Lib USB 1.0
	libusb_device_handle				*dev_handle;
	libusb_context						*ctx;
// Lib Video For Linux 2
	struct v4l2_format					fmt;
	struct v4l2_buffer					buf;
	struct v4l2_requestbuffers			req;
	enum v4l2_buf_type					type;
	fd_set								fds;
	struct timeval						tv;
	int									r, fd = -1;
	unsigned int						i, n_buffers;
	char								*dev_name = "/dev/video0";
	char								out_name[256];
	FILE								*fout;
	struct buffer						*buffers;
	
	unsigned char getMaxValueCounter = 0;

	int error = 0;
	cv::Mat img, blue, red, original, canned, merged;

unsigned char maxValue(cv::Mat img, int type, int diff_red, int diff_blue)
{
	uchar *end = img.dataend, *point = img.data;
	uchar max = 0, red, green, blue;

	if(type == TYPE_GET_MAX_RED)
	{
		while(point != end)
		{
			blue = *point++;
			green = *point++;
			red = *point++;

			if(red >= max && (red - diff_red) > blue && (red - diff_red) > green)
				max = red;
		}
	}
	
	else if(type == TYPE_GET_MAX_BLUE)
	{
		while(point != end)
		{
			blue = *point++;
			green = *point++;
			red = *point++;

			if(blue >= max && (blue - diff_blue) > red && (blue - diff_blue) > green)
				max = blue;
		}
	}

	return max;
}

void thresh(cv::Mat img, cv::Mat dest, int thresh_type, int thresh, int diff_red, int diff_blue)
{
	uchar *end = img.dataend, *point = img.data;
	uchar *dest_point = dest.data;
	static uchar max, blue, green, red;
	
	if(thresh_type == THRESH_TYPE_GET_RED)
	{
		if(getMaxValueCounter++ == 30)
		{
			max = maxValue(img, TYPE_GET_MAX_RED) - thresh;
			getMaxValueCounter = 0;
		}
		while(point != end)
		{
			blue = *point++;
			green= *point++;
			red = *point++;

			if(blue < (red - diff_red) && green < (red - diff_red) && (red - diff_red) >= max)
			{
				*(dest_point++) = 255;
				*(dest_point++) = 255;
				*(dest_point++) = 255;
			}
			else
			{
				*(dest_point++) = 0;
				*(dest_point++) = 0;
				*(dest_point++) = 0;
			}

		}
	}

	else if(thresh_type == THRESH_TYPE_GET_BLUE)
	{
		if(getMaxValueCounter++ == 30)
		{
			max = maxValue(img, TYPE_GET_MAX_BLUE) - thresh;
			getMaxValueCounter = 0;
		}
		while(point != end)
		{
			blue = *point++;
			green= *point++;
			red = *point++;

			if(red < (blue - diff_blue) && green < (blue - diff_blue) && (blue - diff_blue) >= max)
			{
				*(dest_point++) = 255;
				*(dest_point++) = 255;
				*(dest_point++) = 255;
			}
			else
			{
				*(dest_point++) = 0;
				*(dest_point++) = 0;
				*(dest_point++) = 0;
			}

		}
	}
}


void v4l_loop()
{
	clock_t initialTime = clock();
	digitalWrite(0, 0);
	cvtToOpencv();
	original = img.clone();
//	cv::cvtColor(img, img, CV_BGR2HSV);

	blue = img.clone();
	red = img.clone();
	canned = img.clone();
	merged = img.clone();

	thresh(red, red, THRESH_TYPE_GET_RED);
	thresh(blue, blue, THRESH_TYPE_GET_BLUE);

	merge(merged, blue, red);
	cv::Canny(merged, canned, 50, 100);

	error = getDirection(merged, 60);

	legoAct(error);
	digitalWrite(0, 1);

	std::cout << "clocks: " << (int)(clock() - initialTime) << std::endl;
}


void legoAct(int error)
{
	if(dev_handle)
		motorPID(error);
}

void merge(cv::Mat dest, cv::Mat img0, cv::Mat img1)
{
	uchar *temp = dest.data, *tempImg0 = img0.data, *tempImg1 = img1.data, *end = dest.dataend;

	while(temp != end)
	{
		if(*(tempImg0++))
		{
			*(temp++) = 255;
			*(tempImg1++);
		}
		else if(*(tempImg1++))
			*(temp++) = 255;
		else
			*(temp++) = 0;
	}
}

int dir(cv::Mat img, int proportional, float precision)
{
	int direction = 0, jump = 3;
	int counter = 1;
	int limit = 7 * img.rows / 8;

	for(int y = img.rows / 8; y < limit; y += jump)
	{
		for(int x = 0; x < img.cols * 3; x += jump)
		{
			if(img.at<uchar>(y, x) > 50)
			{
				direction += (x - (3 * img.cols / 2));
				counter++;
			}
		}
	}

	return ((int)direction / (proportional * precision * counter));
}

int getDirection(cv::Mat img, int proportional, float precision)
{
	int direction = 0, jump = 3;
	int counter = 1;
	int limit = 7 * img.rows / 8;

	for(int y = img.rows / 8; y < limit; y += jump)
	{
		for(int x = 0; x < img.cols * 3; x += jump)
		{
			if(img.at<uchar>(y, x) > 50)
			{
				direction += (x - (3 * img.cols / 2));
				counter++;
			}
		}
	}

	return ((int)direction / (proportional * precision * counter));
}

bool isRed(uchar* dot)
{
	uchar value = *dot;
	uchar saturation = *(++dot);
	uchar hue = *(++dot);

	return hue <= MAX_HUE_RED && hue >= MIN_HUE_RED &&
		saturation >= MIN_SATURATION_RED && saturation <= MAX_SATURATION_RED &&
		value >= MIN_VALUE_RED && value <= MAX_VALUE_RED;
}

bool isBlue(uchar* dot)
{
	uchar value = *dot;
	uchar saturation = *(++dot);
	uchar hue = *(++dot);

	return hue <= MAX_HUE_BLUE && hue >= MIN_HUE_BLUE &&
		saturation >= MIN_SATURATION_BLUE && saturation <= MAX_SATURATION_BLUE &&
		value >= MIN_VALUE_BLUE && value <= MAX_VALUE_BLUE;
}

void getRed(cv::Mat dest)
{
	uchar* dest_ptr = dest.data;
	uchar* dest_ptr_final = dest.dataend;

	while(dest_ptr != dest_ptr_final)
		if(isRed(dest_ptr)){
			*(dest_ptr++) = 255;
			*(dest_ptr++) = 255;
			*(dest_ptr++) = 255;
		}else{
			*(dest_ptr++) = 0;
			*(dest_ptr++) = 0;
			*(dest_ptr++) = 0;
		}
}

void getBlue(cv::Mat dest)
{
	uchar* dest_ptr = dest.data;
	uchar* dest_ptr_final = dest.dataend;

	while(dest_ptr != dest_ptr_final)
		if(isBlue(dest_ptr)){
			*(dest_ptr++) = 255;
			*(dest_ptr++) = 255;
			*(dest_ptr++) = 255;
		}else{
			*(dest_ptr++) = 0;
			*(dest_ptr++) = 0;
			*(dest_ptr++) = 0;
		}
}

int main()
{
	int r;                                  // for the return values
// Winring PI
	r = wiringPiSetup();
	if(r < 0)
	{
		std::cout << "wiring pi setup error" << std::endl;
		return -1;
	}
	for(int i = 0; i < 8; i++)
		pinMode(i, OUTPUT);
	
	//digitalWrite(1, 1);
	digitalWriteByte(170);
	delay(100);
	digitalWriteByte(85);
	delay(100);
	digitalWriteByte(0);

	r = piHiPri (10); // set program scheduling priority between 0 and 99
	if(r < 0)
		std::cout << "wiring pi could not change program priority" << std::endl;

// MINDSTORMS
	libusb_device **devs;                   // list of devices read
	ctx = NULL;								// session
	ssize_t cnt;                            // number of devices in the list
	r = libusb_init(&ctx);                  // create a session

	if(r < 0)
	{
		std::cout << "Init error" << std::endl;
		#ifdef NXT_ESSENTIAL
			return -1;
		#endif
	}

	libusb_set_debug(ctx, 3);               // set verbosity to level 3
	cnt = libusb_get_device_list(ctx, &devs);       // get the list of devices
	if(cnt < 0)
	{
		std::cout << "Device error" << std::endl;
		#ifdef NXT_ESSENTIAL
			return -1;
		#endif
	}

	dev_handle = libusb_open_device_with_vid_pid(ctx, VENDOR_ID, PRODUCT_ID);
	if(dev_handle == NULL)
		std::cout << "Device Handle Error" << std::endl;
	else
		std::cout << "Device Opened" << std::endl;

	libusb_free_device_list(devs, 1);               //free device list and unref devices in it

	if(libusb_kernel_driver_active(dev_handle, 0) == 1)             //kernel has control over the device. Take him that!
	{
		std::cout << "kernel has control..." << std::endl;
		if(libusb_detach_kernel_driver(dev_handle, 0) == 0)
			std::cout << "...not anymore..." << std::endl;
	}

	if(dev_handle)
		r = libusb_claim_interface(dev_handle, 0);                      //claim interface 0 of device

	if(dev_handle)
		if(r < 0)
		{
			std::cout << "can not claim" << std::endl;
			return -1;
		}

	if(dev_handle)
		std::cout << "Interface claimed" << std::endl;

// Lib Video For Linux
	// OPEN
	fd = v4l2_open(dev_name, O_RDWR | O_NONBLOCK, 0);
	if (fd < 0)
	{
		perror("Cannot open device");
		exit(EXIT_FAILURE);
	}

	// SET FORMAT and DIMM
	CLEAR(fmt);
	fmt.type					= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width			= WIDTH;
	fmt.fmt.pix.height			= HEIGHT;
	fmt.fmt.pix.pixelformat		= V4L2_PIX_FMT_RGB24;
	fmt.fmt.pix.field			= V4L2_FIELD_INTERLACED;
	xioctl(fd, VIDIOC_S_FMT, &fmt);
	if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24)
	{
		std::cout << "Libv4l only accept RGB24 format. Can't proceed.\n";
		exit(EXIT_FAILURE);
	}
	if ((fmt.fmt.pix.width != 640) || (fmt.fmt.pix.height != 480))
		std::cout << "Warning: d";
	else
		std::cout << "D";
	std::cout << "river is sending image at " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height << std::endl;

	// INIT MEMORY MAPPING
	CLEAR(req);
	req.count		= 1;
	req.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory		= V4L2_MEMORY_MMAP;
	xioctl(fd, VIDIOC_REQBUFS, &req);

	// CREATE BUFFER
	buffers = (buffer*) calloc(req.count, sizeof(*buffers));
	for (n_buffers = 0; n_buffers < req.count; ++n_buffers)
	{
		CLEAR(buf);

		// QUERY THE STATUS OF THE BUFFER
		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = n_buffers;
		xioctl(fd, VIDIOC_QUERYBUF, &buf);

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start = v4l2_mmap(NULL, buf.length,
			PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start)
		{
			perror("mmap");
			exit(EXIT_FAILURE);
		}
	}

	// SET CAP QUEUE DEST
	for (i = 0; i < n_buffers; ++i)
	{
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		xioctl(fd, VIDIOC_QBUF, &buf);
	}
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	// START STREAMING
	xioctl(fd, VIDIOC_STREAMON, &type);

	//GET CAP
	for (;;)
	{
		/*do
		{
			FD_ZERO(&fds);
			FD_SET(fd, &fds);

			tv.tv_sec = 2; //delay maximo pra ler a imagem
			tv.tv_usec = 0;

			r = select(fd + 1, &fds, NULL, NULL, &tv);
		} while ((r == -1 && (errno = EINTR)));
		
		if (r == -1)
		{
			perror("select");
			return errno;
		}*/

		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;

		// SAVE A CAP
		xioctl(fd, VIDIOC_DQBUF, &buf);
		saveCap();
		xioctl(fd, VIDIOC_QBUF, &buf);
		
		for(;;)
		{
			// GET CAP
			xioctl(fd, VIDIOC_DQBUF, &buf);
			// "PREPARE" NEXT CAP
			xioctl(fd, VIDIOC_QBUF, &buf);

			v4l_loop();
		}
	}

	// STOP STREAMING
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(fd, VIDIOC_STREAMOFF, &type);
	for (i = 0; i < n_buffers; ++i)
		v4l2_munmap(buffers[i].start, buffers[i].length);

	v4l2_close(fd);

    return 0;
}

void motorPID(int error)
{
	motorMove(OUT_B, initialSpeed + (error * KP));
	motorMove(OUT_C, initialSpeed - (error * KP));
}

int nxtClose()
{
	motorMove(OUT_ABC, 0);

	int r = libusb_release_interface(dev_handle, 0);

	if(r != 0)
	{
			std::cout << "Cannot release interface" << std::endl;
			return r;
	}

	std::cout << "Released interface" << std::endl;
	libusb_close(dev_handle);
	libusb_exit(ctx);

	return 0;
}

/*
 motor = 0x00 -> motor A
 motor = 0x01 -> motor B
 motor = 0x02 -> motor C
 motor = 0xff -> all motors

 -100 < speed < 100

 mode = 0x01 -> motor on
 mode = 0x02 -> brake motor
 mode = 0x04 -> regulated

 regulation = 0x00 -> idle
 regulation = 0x01 -> speed
 regulation = 0x02 -> sync

 -100 < turn < 100 = turn ratio

 runState = 0x00 -> idle
 runState = 0x10 -> ramp up
 runState = 0x20 -> running
 runState = 0x40 -> ramp down

 tachoLimit = 0x00 -> forever
*/
int motorMove(uchar motor, uchar speed, uchar mode, uchar regulation, uchar turn)
{
    unsigned char *data = new unsigned char[9];     //data to send
    int sendAmount = 9;                             //how many data to send
    int actual, r;

    data[0] = 0x80;					// do not wait for response
    data[1] = 0x04;
    data[2] = motor;                //motor
    data[3] = speed;                //speed
    data[4] = mode;                 //mode
    data[5] = regulation;           //regulation mode
    data[6] = turn;                 //turn
    data[7] = 0x20;                 //run state
    data[8] = 0x00;                 //tacho limit

    r = libusb_bulk_transfer(dev_handle, (1 | LIBUSB_ENDPOINT_OUT), data, sendAmount, &actual, 0);

    delete[] data;

    return r;
}

void xioctl(int fh, int request, void *arg)
{
        int r;

        do
		{
                r = v4l2_ioctl(fh, request, arg);
        } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

        if (r == -1)
		{
                fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

void saveCap()
{
	sprintf(out_name, "out000.ppm");
	fout = fopen(out_name, "w");
	if (!fout)
	{
		perror("Cannot open image");
		exit(EXIT_FAILURE);
	}
	fprintf(fout, "P6\n%d %d 255\n",
	fmt.fmt.pix.width, fmt.fmt.pix.height);
	fwrite(buffers[buf.index].start, buf.bytesused, 1, fout);
	fclose(fout);
}

void cvtToOpencv()
{
	uchar *end = ((uchar*)buffers[buf.index].start + buf.bytesused);
	uchar *pointer = ((uchar*)buffers[buf.index].start);
	//char primeiro = 0, segundo = 0, terceiro = 0;
	img = cv::Mat(HEIGHT, WIDTH, CV_8UC3);

	//while(pointer != end)
	//{
		img.data = pointer;//++;
		img.dataend = end;
//			primeiro = (primeiro + *(pointer++)) / 2;
//			segundo = (segundo + *(pointer++)) / 2;
//			terceiro = (terceiro + *(pointer++)) / 2;
	//}
}

