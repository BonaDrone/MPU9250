# 
# Makefile for MPU9250 I2C example
#
#
# Copyright (c) 2018 Simon D. Levy
#
# Library may be used freely and without limit with attribution.

ALL = PassthruTest

SRC = ../../../../src

all: $(ALL)

PassthruTest: PassthruTest.o
	g++ -std=c++11 -o PassthruTest PassthruTest.o -lwiringPi

PassthruTest.o: PassthruTest.cpp 
	g++ -std=c++11 -Wall -I$(SRC) -c PassthruTest.cpp

clean:
	rm -f $(ALL) *.o *~
