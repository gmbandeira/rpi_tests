// g++ Source.cpp `pkg-config opencv --libs --cflags` -lusb-1.0 -lv4l2 -I/usr/local/include -L/usr/local/lib -lwiringPi

// g++ Source.cpp -I/usr/local/include -L/usr/local/lib -lwiringPi 

#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>

int main()
{
/*
	int r = wiringPiSetup();
	if(r < 0)
	{
		std::cout << "error connecting gpio" << std::endl;
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
*/	
	int serialFD = serialOpen("/dev/ttyAMA0", 9600);
	if(serialFD < 0)
	{
		std::cout << "error connecting usb device" << std::endl;
		return -1;
	}
	else
	{
		std::cout << "connected USB device: " << serialFD << std::endl;
	}
	
	digitalWrite(0, 0);
	
	if(wiringPiSetup() == -1)
	{
		std::cout << "error seting up comm" << std::endl;
		return -1;
	}
	else
	{
		std::cout << "comm sucessfully set up" << std::endl;
	}
	
	digitalWrite(0, 1);
	
	while(1)
	{
//		if(serialDataAvail(serialFD) > 0)
//		{
			int inbuffer = serialGetchar(serialFD);
			if(inbuffer == 'x' || inbuffer == -1)
				break;
			
			std::cout << "received: " << inbuffer << std::endl;
//		}
	}
	
	serialClose(serialFD);
	digitalWrite(0, 0);

	return 0;
}



