// g++ Source.cpp `pkg-config opencv --libs --cflags` -lusb-1.0 -lv4l2 -I/usr/local/include -L/usr/local/lib -lwiringPi

#include <wiringPi.h>
#include <libusb-1.0/libusb.h>
#include "my-defines.h"

// Lib USB 1.0
	libusb_device_handle				*dev_handle;
	libusb_context					*ctx;

int main()
{
	libusb_init(&ctx);
	libusb_set_debug(ctx, 3);
	libusb_open_device_with_vid_pid(ctx, VENDOR_ID, PRODUCT_ID);
	libusb_claim_interface(dev_handle, 0);	
	nxtClose();
}

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

int nxtClose()
{
	motorMove(OUT_ABC, 0);

	libusb_release_interface(dev_handle, 0);
	libusb_close(dev_handle);
	libusb_exit(ctx);

	return 0;
}


