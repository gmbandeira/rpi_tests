rpi_tests
=========


To upload a file:

1º git commit
2º git add <file>
3º git push


To remove a file:

1º git rm <file>
2º git commit
3º git push

To compile:

g++ Source.cpp -Wl,-Bstatic `pkg-config opencv --libs --cflags` -Wl,-Bdynamic

===========

Secure Operations:
	1 secure Copy:
		scp README.md pi@192.168.0.169:/home/pi

	2 Secure Move:
	3 Secure Execute:
	4 Secure remove:

================

Dynamic and Static Compile

g++ Source.cpp -Wl,-Bstatic `pkg-config opencv --libs --cflags` -Wl,-Bdynamic
