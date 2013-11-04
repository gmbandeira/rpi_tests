
// time g++ Source.cpp `pkg-config opencv --libs --cflags` -lusb-1.0 -lv4l2 -I/usr/local/include -L/usr/local/lib -lwiringPi -O3 -o source.out

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


void			loop			();

cv::Mat			rotate			(cv::Mat _img, int rotation = 50, int transY = -70, int transZ = 350);
cv::Mat			bin				(cv::Mat _img, std::vector<cv::Point2f> &mc);
void			myThresh		(cv::Mat _img, cv::Mat dest, int thresh_type = THRESH_TYPE_GET_RED, int thresh = 0, int diff_red = 30, int diff_blue = 10);
unsigned char	maxValue		(cv::Mat _img, int type = TYPE_GET_MAX_RED, int diff_red = 200, int diff_blue = 250);
void			merge			(cv::Mat dest, cv::Mat img0, cv::Mat img1);
cv::Mat			rethresh		(std::vector<cv::Point2f> &returned_mc);

void			myMap			(std::vector<cv::Point2f> &mc);
int				getDirection	(cv::Mat _img, int proportional = 80, float precision = 1);

int				main			();

void			legoAct			(int error);
void			motorPID		(int error);
int				motorMove		(unsigned char motor, unsigned char speed = 50, unsigned char mode = 0x01, unsigned char regulation = 0x01, unsigned char turn = 0x90);
int				nxtClose		();

void			xioctl			(int fh, int request, void *arg);
void			saveCap			();
bool			saveCap			(cv::Mat _img, std::string _fileName = "img.ppm");
void			cvtToOpencv		();


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
unsigned int						n_buffers;
char								*dev_name = (char*)"/dev/video0";
char								out_name[256];
FILE								*fout;
struct buffer						*buffers;


int error = 0;
cv::Mat img, gray;
unsigned char getMaxValueCounter = 0;
double thresh = 0;
//clock_t loopTime;


void loop()
{
	cv::Mat rotated, binarized, original;
	std::vector<cv::Point2f> mc;

	cvtToOpencv();
	original = img.clone();

	//rotated = rotate(original);

	rotated = img.clone();
	binarized = bin(rotated, mc);

	//myMap(mc);

	error = getDirection(binarized);

	//digitalWrite(0, 1);
	//legoAct(error);
	//digitalWrite(0, 0);

	//std::cout << clock() - loopTime << std::endl;
	//loopTime = clock();
	std::cout << error << std::endl;
}



cv::Mat rotate(cv::Mat _img, int rotation, int transY, int transZ)
{
	cv::Mat transformed;

	int alpha_=90.;
	alpha_ = rotation;
	int distY_ = transY, distZ_ = transZ;

	cv::rectangle(_img, cv::Point(1, (_img.rows / 2) - 1), cv::Point(_img.cols - 2, _img.rows / 2), cv::Scalar(0, 0, 0), -1, 8, 0);
	cv::rectangle(_img, cv::Point((_img.cols / 2) - 1, 1), cv::Point(_img.cols / 2, _img.rows - 2), cv::Scalar(0, 0, 0), -1, 8, 0);

	double distY, distZ;
	double alpha;

	alpha = ((double)alpha_ - 90.) * CV_PI / 180;
	distY = (double) distY_;
	distZ = (double) distZ_;

	cv::Size taille = _img.size();
	double w = (double)taille.width, h = (double)taille.height;

	// Projection 2D -> 3D matrix
	cv::Mat A1 = (cv::Mat_<double>(4,3) <<
		1, 0, -w/2,
		0, 1, -h/2,
		0, 0,    0,
		0, 0,    1);

	// Rotation matrices around the X,Y,Z axis
	cv::Mat RX = (cv::Mat_<double>(4, 4) <<
		1,          0,           0, 0,
		0, cos(alpha), -sin(alpha), 0,
		0, sin(alpha),  cos(alpha), 0,
		0,          0,           0, 1);

	cv::Mat RY = (cv::Mat_<double>(4, 4) <<
		cos(0), 0, -sin(0), 0,
				0, 1,          0, 0,
		sin(0), 0,  cos(0), 0,
				0, 0,          0, 1);

	cv::Mat RZ = (cv::Mat_<double>(4, 4) <<
		cos(0), -sin(0), 0, 0,
		sin(0),  cos(0), 0, 0,
		0,          0,           1, 0,
		0,          0,           0, 1);

	// Composed rotation matrix with (RX,RY,RZ)
	cv::Mat R = RX * RY * RZ;

	// Translation matrix on the Z axis change dist will change the height
	cv::Mat T = (cv::Mat_<double>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, distY,
		0, 0, 1, distZ,
		0, 0, 0, 1);

	// Camera Intrisecs matrix 3D -> 2D
	cv::Mat A2 = (cv::Mat_<double>(3,4) <<
		500, 0, w/2, 0,
		0, 500, h/2, 0,
		0, 0,   1, 0);

	// Final and overall transformation matrix
	cv::Mat transfo = A2 * (T * (R * A1));

	// Apply matrix transformation
	cv::warpPerspective(_img, transformed, transfo, taille, CV_INTER_CUBIC | CV_WARP_INVERSE_MAP);

	return transformed;
}

cv::Mat bin(cv::Mat _img, std::vector<cv::Point2f> &mc)
{
	cv::Mat blue, red, merged, rethreshed;

	//blur(_img, _img, cv::Size(7,7));

	red = _img.clone();
	blue = _img.clone();
	merged = _img.clone();
	gray = _img.clone();
	rethreshed = _img.clone();

	myThresh(_img, red, THRESH_TYPE_GET_RED);
	myThresh(_img, blue, THRESH_TYPE_GET_BLUE);

	merge(merged, blue, red);

	cv::cvtColor(merged, gray, CV_BGR2GRAY);
	blur(gray, gray, cv::Size(7,7));
	rethreshed = rethresh(mc);

	return rethreshed;
}

void myThresh(cv::Mat _img, cv::Mat dest, int thresh_type, int thresh, int diff_red, int diff_blue)
{
	uchar *end = _img.dataend, *point = _img.data;
	uchar *dest_point = dest.data;
	static uchar max, blue, green, red;
	
	if(thresh_type == THRESH_TYPE_GET_RED)
	{
		if(getMaxValueCounter++ > 30)
		{
			max = maxValue(_img, TYPE_GET_MAX_RED) - thresh;
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
			max = maxValue(_img, TYPE_GET_MAX_BLUE) - thresh;
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

unsigned char maxValue(cv::Mat _img, int type, int diff_red, int diff_blue)
{
	uchar *end = _img.dataend, *point = _img.data;
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

cv::Mat rethresh(std::vector<cv::Point2f> &returned_mc)
{
	cv::Mat canny_output;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	/// Detect edges using canny
	cv::Canny(gray, canny_output, thresh, thresh*3, 3 );
	/// Find contours
	findContours( canny_output, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

	/// Get the moments
	std::vector<cv::Moments> mu(contours.size() );
	for( int i = 0; i < contours.size(); i++ )
	{
		mu[i] = moments( contours[i], false );
	}

	///  Get the mass centers:
	std::vector<cv::Point2f> mc( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
	{
		mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
	}
	returned_mc = mc;

	/// Draw contours
	cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
	for( int i = 0; i < contours.size(); i++ )
	{
		cv::circle( drawing, mc[i], 4, cv::Scalar(255, 255, 255), -1, 8, 0 );
	}

	cv::drawContours(drawing, contours, -1, cv::Scalar(100, 100, 100), -1);

	return drawing;
}



void myMap(std::vector<cv::Point2f> &mc)
{
	int i = mc.size() - 1;
	while(!mc.empty())
	{
		if(mc.data()[i].x > 0.0f && mc.data()[i].x != -1. && mc.data()[i].y != -1. && mc.data()[i].y > 0.0f)
		{
			std::cout << "back:\t" << mc.data()[i].x << "\tfront:\t" << mc.data()[i].y;
			std::cout << "\twidth:\t" << (mc.data()[i].x / 640) * world_width << "\theight:\t" << (mc.data()[i].y / 480) * world_height << std::endl;
		}
		mc.pop_back();
		i--;
	}

	//cv::circle( img, cv::Point(h, w/3), 6, cv::Scalar(0, 100, 0), -1);

	std::cout << "\n\n\n";
}

int getDirection(cv::Mat _img, int proportional, float precision)
{
	int direction = 0, jump = 3;
	int counter = 1;
	int limit = 7 * _img.rows / 8;

	for(int y = _img.rows / 8; y < limit; y += jump)
	{
		for(int x = 0; x < _img.cols * 3; x += jump)
		{
			if(_img.at<uchar>(y, x) != 0)
			{
				direction += (x - (3 * _img.cols / 2));
				counter++;
			}
		}
	}

	return ((int)direction / (proportional * precision * counter));
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

	r = libusb_claim_interface(dev_handle, 0);                      //claim interface 0 of device

	if(r < 0)
	{
		std::cout << "can not claim" << std::endl;
		#ifdef NXT_ESSENTIAL
			return -1;
		#endif
	}

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
	for (int i = 0; i < n_buffers; ++i)
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

			loop();
		}
	}

	// STOP STREAMING
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(fd, VIDIOC_STREAMOFF, &type);
	for (int i = 0; i < n_buffers; ++i)
		v4l2_munmap(buffers[i].start, buffers[i].length);

	v4l2_close(fd);

    return 0;
}



void legoAct(int error)
{

	motorPID(error);
}

void motorPID(int error)
{
	motorMove(OUT_B, (1) * (initialSpeed + (error * KP)));
	motorMove(OUT_C, (1) * (initialSpeed - (error * KP)));
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

    data[0] = 0x80;
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



void xioctl(int fh, int request, void *arg)
{
        int r;

        do {
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

bool saveCap(cv::Mat _img, std::string _fileName)
{
	std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
	
	return cv::imwrite(_fileName, _img, compression_params);
}

void cvtToOpencv()
{
	uchar *end = ((uchar*)buffers[buf.index].start + buf.bytesused);
	uchar *pointer = ((uchar*)buffers[buf.index].start);
	img = cv::Mat(HEIGHT, WIDTH, CV_8UC3);

	img.data = pointer;
	img.dataend = end;
}

