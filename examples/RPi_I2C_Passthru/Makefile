# 
# Makefile for MPU9250 I2C example
#
#
# Copyright (c) 2018 Simon D. Levy
#
# Library may be used freely and without limit with attribution.

ALL = RPi_I2C_Passthru

SRC = ../../src

all: $(ALL)

RPi_I2C_Passthru: RPi_I2C_Passthru.o
	g++ -std=c++11 -o RPi_I2C_Passthru RPi_I2C_Passthru.o -lwiringPi

RPi_I2C_Passthru.o: RPi_I2C_Passthru.cpp 
	g++ -std=c++11 -Wall -I$(SRC) -c RPi_I2C_Passthru.cpp
clean:
	rm -f $(ALL) *.o *~
