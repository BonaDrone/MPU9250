# 
# Makefile for MPU6050 WiringPi examples
#
# Copyright (c) 2018 Simon D. Levy
#
# This file is part of MPU6050.
#
# MPU6050 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# MPU6050 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with MPU6050.  If not, see <http://www.gnu.org/licenses/>.

  
# Change this to whereever you installed CrossPlatformI2C
CPI2C = $(HOME)/CrossPlatformI2C/

ALL = Basic

MPUSRC = ../../../src

I2CINC = $(CPI2C)/src
I2CSRC = $(CPI2C)/extras/wiringpi/src

all: $(ALL)

Basic: Basic.o MPU.o MPU6050.o WiringPiI2C.o main.o
	g++ -std=c++11 -o Basic Basic.o MPU.o MPU6050.o WiringPiI2C.o main.o -lwiringPi

Basic.o: Basic.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(MPUSRC) -c Basic.cpp

MPU.o:  $(MPUSRC)/MPU.cpp  $(MPUSRC)/MPU.h
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU.cpp

MPU6050.o:  $(MPUSRC)/MPU6050.cpp  $(MPUSRC)/MPU6050.h
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(MPUSRC) -c $(MPUSRC)/MPU6050.cpp

WiringPiI2C.o: $(I2CSRC)/WiringPiI2C.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -c $(I2CSRC)/WiringPiI2C.cpp

main.o: main.cpp 
	g++ -std=c++11 -Wall -c main.cpp

run: Basic
	./Basic

edit:
	vim Basic.cpp

clean:
	rm -f $(ALL) *.o *~
