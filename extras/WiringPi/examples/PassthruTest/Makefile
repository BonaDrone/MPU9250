# 
# Makefile for MPU9250 WiringPi I2C pass-through-mode example
#
#
# Copyright (c) 2018 Simon D. Levy
#
# Library may be used freely and without limit with attribution.

ALL = PassthruTest

MPUSRC = ../../../../src
RPISRC = ../../src

all: $(ALL)

PassthruTest: PassthruTest.o MPU9250.o WiringPiTransfer.o
	g++ -std=c++11 -o PassthruTest PassthruTest.o MPU9250.o WiringPiTransfer.o -lwiringPi

PassthruTest.o: PassthruTest.cpp 
	g++ -std=c++11 -Wall -I$(RPISRC) -I$(MPUSRC) -c PassthruTest.cpp

MPU9250.o: $(MPUSRC)/MPU9250.cpp 
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU9250.cpp

WiringPiTransfer.o: $(RPISRC)/WiringPiTransfer.cpp 
	g++ -std=c++11 -Wall -I$(RPISRC) -I$(MPUSRC) -c $(RPISRC)/WiringPiTransfer.cpp

run: PassthruTest
	./PassthruTest

edit:
	vim PassthruTest.cpp

clean:
	rm -f $(ALL) *.o *~
