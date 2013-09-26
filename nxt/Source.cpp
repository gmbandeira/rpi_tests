//#define _NONE0
#ifdef _NONE0
// NXT++ 0.6
#include "nxtpp_07\include\NXT++.h"
#pragma comment (lib, "nxtpp_07/lib/fantom.lib" )

#include <opencv2/opencv.hpp>
#include <iostream>
#include <WinSock2.h>
#include <ws2bth.h>
#include <bthsdpdef.h>
#include <bluetoothapis.h>


int dir(cv::Mat img, int precision = 10, int proportional = 150)
{
	int direction = 0, jump = ((3 * img.rows / 4) - (img.rows / 4)) / precision;
	//int startingLine;
	int var = (int)(0.3 * img.cols);

	for(int y = img.rows / 4; y < 3 * img.rows / 4; y += jump)
	{
		for(int x = (img.cols / 2) - var; x < (img.cols / 2) + var; x++)
		{
			if(img.at<uchar>(y, x) > 50)
			{
				direction += (x - (img.cols / 2));
			}
		}
	}

	return direction / (proportional * precision);
}

int getInitial(cv::Mat img)
{
	

	return 0;
}

void cp(cv::Mat img)
{
	for(int i = 0; i < img.cols - 1; i++)
	{
		for(int j = 0; j < img.rows - 1; j++)
		{
			if(img.at<uchar>(j, i) == 255)
				img.at<uchar>(j, i) = 0;
		}
	}
}

void fill(cv::Mat img)
{
	int val = 0;
	cv::Mat comp = img;
	cp(comp);
	for(int i = 0; i < img.cols; i ++)
	{
		if(img.at<uchar>(0, i) == val)
			cv::floodFill(img, comp, cv::Point(i, 0), val);
		if(img.at<uchar>(img.rows - 1, i) == val)
			cv::floodFill(img, comp, cv::Point(i, img.rows - 1), val);
	}

	for(int i = 0; i < img.rows; i ++)
	{
		if(img.at<uchar>(i, 0) == val)
			cv::floodFill(img, comp,cv::Point(0, i), val);
		if(img.at<uchar>(i, img.rows - 1) == val)
			cv::floodFill(img, comp, cv::Point(img.rows - 1, i), val);
	}
}

void fill2(cv::Mat img)
{
	cv::dilate(img, img, cv::Mat());
}



/**
 * Perform one thinning iteration.
 * Normally you wouldn't call this function directly from your code.
 *
 * @param  im    Binary image with range = 0-1
 * @param  iter  0=even, 1=odd
 */
void thinningIteration(cv::Mat& im, int iter)
{
    cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

    for (int i = 1; i < im.rows-1; i++)
    {
        for (int j = 1; j < im.cols-1; j++)
        {
            uchar p2 = im.at<uchar>(i-1, j);
            uchar p3 = im.at<uchar>(i-1, j+1);
            uchar p4 = im.at<uchar>(i, j+1);
            uchar p5 = im.at<uchar>(i+1, j+1);
            uchar p6 = im.at<uchar>(i+1, j);
            uchar p7 = im.at<uchar>(i+1, j-1);
            uchar p8 = im.at<uchar>(i, j-1);
            uchar p9 = im.at<uchar>(i-1, j-1);

            int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) + 
                     (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) + 
                     (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                     (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                marker.at<uchar>(i,j) = 1;
        }
    }

    im &= ~marker;
}

/**
 * Function for thinning the given binary image
 *
 * @param  im  Binary image with range = 0-255
 */
void thinning(cv::Mat& im)
{
    im /= 255;

    cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(im, 0);
        thinningIteration(im, 1);
        cv::absdiff(im, prev, diff);
        im.copyTo(prev);
    } 
    while (cv::countNonZero(diff) > 0);

    im *= 255;
}

Comm::NXTComm nxtInit()
{
	std::cout << "Searching for NXT devices... (please wait) \n\n";
	/*
	std::vector<std::vector<std::string> > devicelist;
	devicelist = Comm::ListNXTDevices(true);

	std::cout << "Search complete! \n\n";

	for (unsigned int i = 0; i < devicelist.size(); i++)
	{
		std::cout << "NXT Device " << i << "   Name: " << devicelist[i][0] << "   MAC Address: " << devicelist[i][1] << std::endl;
	}
	std::cout << std::endl << std::endl << std::endl << std::endl;
	*/
	Comm::NXTComm comm1;

	if (NXT::OpenNXTDevice(&comm1, "001653146C1D", true))
	{
		std::cout << "\nConnected to brick... \n";

		// Get NXT device information
		double protocol = NXT::GetProtocolVersion(&comm1);
		double version = NXT::GetFirmwareVersion(&comm1);
		int battery = NXT::BatteryLevel(&comm1);
		std::string name = NXT::GetName(&comm1);

		// Output information to console
		std::cout << "Protocol version: " << protocol << std::endl; // e.g. 1.124000 or 
		std::cout << "Firmware version: " << version << std::endl; // e.g. 1.280000 or 1.310000
		std::cout << "Battery Level: " << battery << std::endl; //e.g.  8198, or 6674 when battery low warning
		std::cout << "Name: " << name.data() << std::endl; // NXT device name

		return comm1;
	}
	else
	{
		std::cout << "connection error" << std::endl;
		system("pause");
	}
}

void nxtMoveMotors(Comm::NXTComm* comm1, int speedB = 20, int speedC = 20)
{
	//if ((*comm1).Open())
	//{
		NXT::Motor::SetForward(comm1, OUT_B, speedB);
		NXT::Motor::SetForward(comm1, OUT_C, speedC);
	//}
}

void nxtClose(Comm::NXTComm* comm1)
{
	//if((*comm1).Open())
		NXT::Close(comm1); //close the NXT
}

void nxtStopAllMotors(Comm::NXTComm* comm1)
{
	NXT::Motor::BrakeOn(comm1, OUT_A);
	NXT::Motor::BrakeOn(comm1, OUT_B);
	NXT::Motor::BrakeOn(comm1, OUT_C);
}

int main()
{
	//cv::VideoCapture cap("C:\\Users\\Admin\\Documents\\gmb\\Dropbox\\GPRT\\projects\\nxt_follow_line\\nxt_follow_line\\cap.wmv");
	cv::VideoCapture cap(0);
	Comm::NXTComm comm1 = nxtInit();
	int turn = 0;
	//CV_CAP_PROP_FRAME_COUNT // total frames
	//CV_CAP_PROP_POS_MSEC // current millis()
	//CV_CAP_PROP_FPS //frame rate
	//double endTime = cap.get(CV_CAP_PROP_FRAME_COUNT) * 1000 / cap.get(CV_CAP_PROP_FPS);
	
	if(!cap.isOpened())
	{
		system("pause");
		std::cout << "cap open error" << std::endl;
		return -1;
	}

	cv::Mat edges, orig, frame;
	cv::Mat eroded, temp, skel(frame.size(), CV_8UC1, cv::Scalar(0));

	cv::namedWindow("edges", 1);
	cv::namedWindow("orig", 1);
	float timeLast;

	for(;;)
	{
		//timeLast = endTime - cap.get(CV_CAP_PROP_POS_MSEC);
		cap >> frame;
		orig = frame;
		cvtColor(frame, edges, CV_BGR2GRAY);
		GaussianBlur(edges, edges, cv::Size(7 ,7), 1.5, 1.5);
		//cv::medianBlur(edges, edges, 7);
		//Canny(edges, edges, 210, 255);
		cv::threshold(edges, edges, 65, 255, cv::THRESH_BINARY_INV);
		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		for(int i = 0; i < 5; i ++)
		{
			cv::dilate(edges, edges, element, cv::Point(-1, -1), 2);
			cv::erode(edges, edges, element, cv::Point(-1, -1), 8);
		}
		/*do{
		cv::erode(edges, eroded, element);
		cv::dilate(eroded, temp, element);
		cv::absdiff(edges, temp, temp);
		skel = edges | skel;
		edges = eroded;

		//cv::subtract(skel, temp, skel);
		//eroded.copyTo(edges);
		}while(cv::countNonZero(edges) != 0);*/
		//thinning(edges);
		//Canny(edges, edges, 210, 255);
		//cv::blur(edges, edges, cv::Size(7, 7));
		//fill2(edges);
		//fill(edges);
		turn = dir(edges, 50, 350);
		//std::cout << turn << "\t" << timeLast << std::endl;
		nxtMoveMotors(&comm1, 10 - turn, 10 + turn);
		imshow("edges", edges);
		imshow("orig", orig);
		//if(cv::waitKey(30) >= 0 || timeLast < 1200) break;
		if(cv::waitKey(30) >= 0) break;
	}

	//system("pause");
	nxtStopAllMotors(&comm1);
	nxtClose(&comm1);
	return 0;
}

#endif

#define _NONE1
#ifdef _NONE1

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

	#if defined LINUX
		#include <sys/ioctl.h>
		#include <sys/types.h>
		#include <sys/time.h>
		#include <sys/mman.h>
		#include <linux/videodev2.h>
		#include "libv4l2.h"
		#include <linux/v4l2-mediabus.h>

		#include <libusb-1.0/libusb.h>
	#elif _WIN32
		#include "nxtpp_07/include/NXT++.h"
		#include "nxtpp_07/include/comm.h"
		#include "nxtpp_07/include/visatype.h"
		#pragma comment (lib, "nxtpp_07/lib/fantom.lib" )
	#endif

#include "my-defines.h"


//#define	PRINT_SECONDS
#define	PRINT_CLOCKS
#define	PRINT_ERROR

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
void	motorMove		(int out, int speed);
int		motorMove		(uchar motor, uchar speed, uchar mode, uchar regulation, uchar turn);
int		nxtClose		();



cv::VideoCapture cap(DEVICE_NAME);
//dlib_lego lego();


#ifdef _WIN32
	Comm::NXTComm				comm;
#else
	libusb_device_handle		*dev_handle;
	libusb_context				*ctx;
#endif


void loop()
{
	clock_t getCapTime, toHSVTime, getRedTime, getBlueTime, mergeTime, getTargetTime;
	int error = 0;
	cv::Mat img, blue, red, original;

	//CV_CAP_PROP_FRAME_COUNT	// total frames
	//CV_CAP_PROP_POS_MSEC		// current millis()
	//CV_CAP_PROP_FPS			//frame rate
	double endTime = cap.get(CV_CAP_PROP_FRAME_COUNT) * 1000 / cap.get(CV_CAP_PROP_FPS);
	float timeLast;

	cv::namedWindow("red");
	cv::namedWindow("blue");
	cv::namedWindow("original");
	cv::namedWindow("merged");

	while(1)
	{
		timeLast = endTime - cap.get(CV_CAP_PROP_POS_MSEC);

		getCapTime = clock();
		cap >> img;
		original = img.clone();
		getCapTime = clock() - getCapTime;

		toHSVTime = clock();
		cv::cvtColor(img, img, CV_BGR2HSV);
		toHSVTime = clock() - toHSVTime;

		blue = img.clone();
		red = img.clone();

		getBlueTime = clock();
		getBlue(blue);
		getBlueTime = clock() - getBlueTime;

		getRedTime = clock();
		getRed(red);
		getRedTime = clock() - getRedTime;

		mergeTime = clock();
		merge(img, red, blue);
		mergeTime = clock() - mergeTime;

		getTargetTime = clock();
		error = getDirection(img, 60);
		getTargetTime = clock() - getTargetTime;

		legoAct(error);

		cv::imshow("original", original);
		cv::imshow("red", red);
		cv::imshow("blue", blue);
		cv::imshow("merged", img);

#ifdef PRINT_SECONDS
		std::cout << std::endl << "getCap: " << ((float)getCapTime)/CLOCKS_PER_SEC << "\ttransform: " << ((float)toHSVTime)/CLOCKS_PER_SEC 
			<< "\tgetRed: " << ((float)getRedTime)/CLOCKS_PER_SEC << "\tgetTarget: " << ((float)getTargetTime)/CLOCKS_PER_SEC;

#elif defined PRINT_CLOCKS
		std::cout << std::endl << "getCap: " << ((float)getCapTime) << "\ttransform: " << ((float)toHSVTime)
			<< "\tgetRed: " << ((float)getRedTime) << "\tgetTarget: " << ((float)getTargetTime);
#endif

		std::cout << "\tError: " << error << std::endl;

		if(cvWaitKey(30) > 0 || timeLast < 1100) break;
	}
}


void legoAct(int error)
{

	motorPID(error);
}


void merge(cv::Mat dest, cv::Mat img0, cv::Mat img1)
{
	uchar *temp = dest.data, *tempImg0 = img0.data, *tempImg1 = img1.data;
	while(temp != dest.dataend)
	{
		if(*(tempImg0++) || *(tempImg1++))
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
	uchar hue = *dot;
	uchar saturation = *(++dot);

	return hue <= MAX_HUE_RED && hue >= MIN_HUE_RED &&
		saturation >= MIN_SATURATION_RED && saturation <= MAX_SATURATION_RED;
}


bool isBlue(uchar* dot)
{
	uchar hue = *dot;
	uchar saturation = *(++dot);

	return hue <= MAX_HUE_BLUE && hue >= MIN_HUE_BLUE &&
		saturation >= MIN_SATURATION_BLUE && saturation <= MAX_SATURATION_BLUE;
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
// OPEN CV
        if(!cap.isOpened())
        {
                std::cout << "cap open error" << std::endl;
				#ifdef _WIN32
					system("pause");
				#endif
                return -1;
        }

// MINDSTORMS
#ifdef _WIN32
	if (NXT::OpenNXTDevice(&comm, "001653146C1D", true))
	{
		std::cout << "\nConnected to brick... \n";

		// Get NXT device information
		double protocol = NXT::GetProtocolVersion(&comm);
		double version = NXT::GetFirmwareVersion(&comm);
		int battery = NXT::BatteryLevel(&comm);
		std::string name = NXT::GetName(&comm);

		// Output information to console
		std::cout << "Protocol version: " << protocol << std::endl; // e.g. 1.124000 or 
		std::cout << "Firmware version: " << version << std::endl; // e.g. 1.280000 or 1.310000
		std::cout << "Battery Level: " << battery << std::endl; //e.g.  8198, or 6674 when battery low warning
		std::cout << "Name: " << name.data() << std::endl; // NXT device name

		NXT::PlayTone  (&comm, 1000, 100 ); // Play high tone from NXT
	}
	else
	{
		std::cout << "connection error" << std::endl;
		system("pause");
		return -1;
	}

#elif LINUX
	libusb_device **devs;                   // list of devices read
	ctx = NULL;								// session
	int r;                                  // for the return values
	ssize_t cnt;                            // number of devices in the list
	r = libusb_init(&ctx);                  // create a session

	if(r < 0)
	{
			std::cout << "Init error" << std::endl;
			return -1;
	}

	libusb_set_debug(ctx, 3);               // set verbosity to level 3
	cnt = libusb_get_device_list(ctx, &devs);       // get the list of devices
	if(cnt < 0)
	{
			std::cout << "Device error" << std::endl;
			return -1;
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
			return -1;
	}

	std::cout << "Interface claimed" << std::endl;
#endif

		loop();

        return 0;
}

void motorPID(int error)
{
	motorMove(OUT_B, initialSpeed + (error * KP));
	motorMove(OUT_C, initialSpeed - (error * KP));
}

int nxtClose()
{
#ifdef _WIN32
	//if((*comm1).Open())
	//{
		NXT::Motor::BrakeOn(&comm, OUT_A);
		NXT::Motor::BrakeOn(&comm, OUT_B);
		NXT::Motor::BrakeOn(&comm, OUT_C);		
		NXT::Close(&comm);
	//}
#elif LINUX
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
#endif
	return 0;
}

#ifdef _WIN32
void motorMove(int out, int speed)
{
	//if (comm.Open())
	//{
		NXT::Motor::SetForward(&comm, out, speed);
	//}
}

#elif LINUX
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

    data[0] = 0x00;
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

#endif

#endif
