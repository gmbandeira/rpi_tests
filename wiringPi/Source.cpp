#include <wiringPi.h>

int main()
int r = wiringPiSetup();
if(r < 0)
{
std::cout << "error" << std::endl;
return -1;
}
for(int i = 0; i < 8; i++)
		pinMode(i, OUTPUT);
	
	//digitalWrite(1, 1);
	digitalWriteByte(B10101010);
	delay(100);
	digitalWriteByte(B01010101);
	delay(100);
	digitalWriteByte(B00000000);

main();
return 0;
}
