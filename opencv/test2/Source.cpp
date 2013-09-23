/*
 * Source.cpp
 *
 *  Created on: 23/09/2013
 *      Author: gmbandeira
 */

#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

int main( int argc, const char** argv )
{
	VideoCapture cap(0);
	if(!cap.isOpened())
	{
		cout << "cap error";
		return -1;
	}
	Mat img;
	while(1)
	{
		cap >> img;
		blur(img, img, Size(3, 3));
		Canny(img, img, 1, 3, 3);
		imwrite("img.png", img);
	}
}
