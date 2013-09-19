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

g++ Source.cpp -static `pkg-config opencv --libs --cflags` -dynamic
