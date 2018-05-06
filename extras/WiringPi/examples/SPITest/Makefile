# 
# Makefile for MPU9250 WiringPi SPI example
#
#
# Copyright (c) 2018 Simon D. Levy
#
# Library may be used freely and without limit with attribution.

ALL = SPITest

MPUSRC = ../../../../src
RPISRC = ../../src

all: $(ALL)

SPITest: SPITest.o MPU9250.o WiringPiTransfer.o
	g++ -std=c++11 -o SPITest SPITest.o MPU9250.o WiringPiTransfer.o -lwiringPi

SPITest.o: SPITest.cpp 
	g++ -std=c++11 -Wall -I$(RPISRC) -I$(MPUSRC) -c SPITest.cpp

MPU9250.o: $(MPUSRC)/MPU9250.cpp 
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU9250.cpp

WiringPiTransfer.o: $(RPISRC)/WiringPiTransfer.cpp 
	g++ -std=c++11 -Wall -I$(RPISRC) -I$(MPUSRC) -c $(RPISRC)/WiringPiTransfer.cpp

run: SPITest
	./SPITest

edit:
	vim SPITest.cpp

clean:
	rm -f $(ALL) *.o *~
