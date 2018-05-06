# 
# Makefile for MPU9250 I2C example
#
#
# Copyright (c) 2018 Simon D. Levy
#
# Library may be used freely and without limit with attribution.

ALL = MasterTest

MPUSRC = ../../../../src
RPISRC = ../../src

all: $(ALL)

MasterTest: MasterTest.o MPU9250.o WiringPiTransfer.o
	g++ -std=c++11 -o MasterTest MasterTest.o MPU9250.o WiringPiTransfer.o -lwiringPi

MasterTest.o: MasterTest.cpp 
	g++ -std=c++11 -Wall -I$(RPISRC) -I$(MPUSRC) -c MasterTest.cpp

MPU9250.o: $(MPUSRC)/MPU9250.cpp 
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU9250.cpp

WiringPiTransfer.o: $(RPISRC)/WiringPiTransfer.cpp 
	g++ -std=c++11 -Wall -I$(RPISRC) -I$(MPUSRC) -c $(RPISRC)/WiringPiTransfer.cpp

clean:
	rm -f $(ALL) *.o *~
