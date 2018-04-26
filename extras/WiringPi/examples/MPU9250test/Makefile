# 
# Makefile for MPU9250 SPI example
#
#
# Copyright (c) 2016 Simon D. Levy
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
# and associated documentation files (the "Software"), to deal in the Software without restriction, 
# including without limitation the rights to use, copy, modify, merge, publish, distribute, 
# sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or 
# substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

ALL = MPU9250test simpletest

SRC1 = ../../src
SRC2 = ../../../../src

all: $(ALL)

MPU9250test: MPU9250test.o 
	g++ -std=c++11 -o MPU9250test MPU9250test.o -lwiringPi -lmpu9250 

MPU9250test.o: MPU9250test.cpp $(SRC1)/WiringPiTransfer.h $(SRC2)/ByteTransfer.h 
	g++ -std=c++11 -Wall -I$(SRC1) -I$(SRC2) -c MPU9250test.cpp

run: MPU9250test
	./MPU9250test

simple: simpletest
	./simpletest

simpletest: simpletest.o 
	g++ -std=c++11 -o simpletest simpletest.o -lwiringPi -lmpu9250 

simpletest.o: simpletest.cpp $(SRC1)/WiringPiTransfer.h $(SRC2)/ByteTransfer.h 
	g++ -std=c++11 -Wall -I$(SRC1) -I$(SRC2) -c simpletest.cpp

clean:
	rm -f $(ALL) *.o *~
