# 
# Makefile for MPU6050 I2CDEV examples
#
# Copyright (c) 2018 Simon D. Levy
#
# This file is part of MPU6050.
#
# Library may be used freely and without limit with attribution.
  
# Change this to whereever you installed CrossPlatformI2C
CPI2C = $(HOME)/CrossPlatformI2C/

ALL = Basic

MPUSRC = ../../../src

I2CINC = $(CPI2C)/src
I2CSRC = $(CPI2C)/extras/i2cdev/src

all: $(ALL)

Basic: Basic.o MPU.o MPU6050.o I2CDevI2C.o main.o
	g++ -std=c++11 -o Basic Basic.o MPU.o MPU6050.o I2CDevI2C.o main.o

Basic.o: Basic.cpp 
	g++ -std=c++11 -Wall -I$(MPUSRC) -c Basic.cpp

MPU6050.o: $(MPUSRC)/MPU6050.cpp  $(MPUSRC)/MPU6050.h
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(MPUSRC) -c $(MPUSRC)/MPU6050.cpp

MPU.o: $(MPUSRC)/MPU.cpp  $(MPUSRC)/MPU.h
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU.cpp

I2CDevI2C.o: $(I2CSRC)/I2CDevI2C.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -c $(I2CSRC)/I2CDevI2C.cpp

main.o: main.cpp 
	g++ -std=c++11 -Wall -c main.cpp

run: Basic
	./Basic

edit:
	vim Basic.cpp

clean:
	rm -f $(ALL) *.o *~
