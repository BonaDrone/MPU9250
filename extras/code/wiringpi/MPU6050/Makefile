# 
# Makefile for MPU6050 WiringPi examples
#
# Copyright (c) 2018 Simon D. Levy
#
# This file is part of MPU.
#
# MPU is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# MPU is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with MPU.  If not, see <http://www.gnu.org/licenses/>.

  
# Change this to whereever you installed CrossPlatformDataBus
CPDB = $(HOME)/CrossPlatformDataBus

ALL = Basic

MPUSRC = ../../../../src

CPSRC  = $(CPDB)/src
CPEXT  = $(CPDB)/extras
CPCMN  = $(CPEXT)/common
I2CSRC = $(CPEXT)/i2c/wiringpi/src

all: $(ALL)

Basic: Basic.o MPU.o MPU6xx0.o MPU6050.o WiringPiI2C.o main.o errmsg.o
	g++ -std=c++11 -o Basic Basic.o MPU.o MPU6xx0.o MPU6050.o WiringPiI2C.o main.o errmsg.o -lwiringPi

Basic.o: Basic.cpp 
	g++ -std=c++11 -Wall -I$(CPCMN) -I$(MPUSRC) -c Basic.cpp

MPU.o:  $(MPUSRC)/MPU.cpp  $(MPUSRC)/MPU.h
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU.cpp

MPU6xx0.o:  $(MPUSRC)/MPU6xx0.cpp  $(MPUSRC)/MPU6xx0.h
	g++ -std=c++11 -Wall -I$(I2CSRC) -I$(MPUSRC) -c $(MPUSRC)/MPU6xx0.cpp

MPU6050.o:  $(MPUSRC)/MPU6050.cpp  $(MPUSRC)/MPU6050.h
	g++ -std=c++11 -Wall -I$(CPSRC) -I$(MPUSRC) -c $(MPUSRC)/MPU6050.cpp

WiringPiI2C.o: $(I2CSRC)/WiringPiI2C.cpp 
	g++ -std=c++11 -Wall -I$(CPSRC) -c $(I2CSRC)/WiringPiI2C.cpp

main.o: $(CPCMN)/main.cpp 
	g++ -std=c++11 -Wall -c $(CPCMN)/main.cpp

errmsg.o: $(CPCMN)/errmsg.cpp 
	g++ -std=c++11 -Wall -c $(CPCMN)/errmsg.cpp

run: Basic
	./Basic

edit:
	vim Basic.cpp

clean:
	rm -f $(ALL) *.o *~
