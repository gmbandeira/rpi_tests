// g++ Source.cpp `pkg-config opencv --libs --cflags` -lusb-1.0 -lv4l2 -I/usr/local/include -L/usr/local/lib -lwiringPi

// g++ Source.cpp -I/usr/local/include -L/usr/local/lib -lwiringPi

#include <wiringPi.h>
#include <iostream>

int main()
{
int r = wiringPiSetup();
if(r < 0)
{
std::cout << "error" << std::endl;
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

main();
return 0;
}
