#   Makefile for MPU6050 I2CDEV examples
#  
#   Copyright 
#
#   This file is part of MPU.
#
#   MPU is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   MPU is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#   You should have received a copy of the GNU General Public License
#   along with MPU.  If not, see <http://www.gnu.org/licenses/>.
  
# Change this to whereever you installed CrossPlatformI2C
CPI2C = $(HOME)/CrossPlatformI2C/

ALL = Basic

MPUSRC = ../../../src

I2CINC = $(CPI2C)/src
I2CSRC = $(CPI2C)/extras/i2cdev/src

all: $(ALL)

Basic: Basic.o MPU.o MPU60x0.o MPU6050.o I2CDevI2C.o main.o
	g++ -std=c++11 -o Basic Basic.o MPU.o MPU60x0.o MPU6050.o I2CDevI2C.o main.o

Basic.o: Basic.cpp 
	g++ -std=c++11 -Wall -I$(MPUSRC) -c Basic.cpp

MPU6050.o: $(MPUSRC)/MPU6050.cpp  $(MPUSRC)/MPU6050.h
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(MPUSRC) -c $(MPUSRC)/MPU6050.cpp

MPU60x0.o: $(MPUSRC)/MPU60x0.cpp  $(MPUSRC)/MPU60x0.h
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(MPUSRC) -c $(MPUSRC)/MPU60x0.cpp

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
