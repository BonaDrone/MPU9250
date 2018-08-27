# 
# Makefile for MPU9250 WiringPi I^2C examples
#
# Copyright (c) 2018 Simon D. Levy
#
# This file is part of MPU9250.
#
# Library may be used freely and without limit with attribution.
  
# Change this to whereever you installed CrossPlatformI2C
CPI2C = $(HOME)/CrossPlatformI2C/

ALL = MasterTest PassthruTest

MPUSRC = ../../../../src/

I2CINC = $(CPI2C)/src
I2CSRC = $(CPI2C)/extras/wiringpi/src

all: $(ALL)

MasterTest: MasterTest.o MPU9250.o MPU9250_I2C.o WiringPiI2C.o main.o
	g++ -std=c++11 -o MasterTest MasterTest.o MPU9250.o MPU9250_I2C.o WiringPiI2C.o main.o -lwiringPi

MasterTest.o: MasterTest.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(MPUSRC) -c MasterTest.cpp

PassthruTest: PassthruTest.o MPU9250.o MPU9250_I2C.o WiringPiI2C.o main.o
	g++ -std=c++11 -o PassthruTest PassthruTest.o MPU9250.o MPU9250_I2C.o WiringPiI2C.o main.o -lwiringPi

PassthruTest.o: PassthruTest.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(MPUSRC) -c PassthruTest.cpp

MPU9250.o: $(MPUSRC)/MPU9250.cpp 
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU9250.cpp

MPU9250_I2C.o: $(MPUSRC)/MPU9250_I2C.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(MPUSRC) -c $(MPUSRC)/MPU9250_I2C.cpp

WiringPiI2C.o: $(I2CSRC)/WiringPiI2C.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -c $(I2CSRC)/WiringPiI2C.cpp

main.o: ../../../main.cpp 
	g++ -std=c++11 -Wall -c ../../../main.cpp

run: MasterTest
	./MasterTest

edit:
	vim MasterTest.cpp

clean:
	rm -f $(ALL) *.o *~
