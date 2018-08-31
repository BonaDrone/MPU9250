#   Makefile for MPU9250 WiringPi SPI examples
#  
#   Copyright (C) 2018 Simon D. Levy
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
  
# Change this to whereever you installed CrossPlatformDataBus
CPDB = $(HOME)/CrossPlatformDataBus/

ALL = SPITest

MPUSRC = ../../../../../src

SPIINC = $(CPDB)/src
SPISRC = $(CPDB)/extras/spi/wiringpi/src

all: $(ALL)

SPITest: SPITest.o MPU.o MPU9250.o MPU9250_Master.o MPU9250_SPI.o WiringPiSPI.o main.o
	g++ -std=c++11 -o SPITest SPITest.o MPU.o MPU9250.o MPU9250_Master.o MPU9250_SPI.o WiringPiSPI.o main.o -lwiringPi

SPITest.o: SPITest.cpp 
	g++ -std=c++11 -Wall -I$(SPIINC) -I$(MPUSRC) -c SPITest.cpp

MPU.o: $(MPUSRC)/MPU.cpp 
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU.cpp

MPU9250.o: $(MPUSRC)/MPU9250.cpp 
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU9250.cpp

MPU9250_Master.o: $(MPUSRC)/MPU9250_Master.cpp 
	g++ -std=c++11 -Wall -D_Master -I$(MasterINC) -I$(MPUSRC) -c $(MPUSRC)/MPU9250_Master.cpp

MPU9250_SPI.o: $(MPUSRC)/MPU9250_SPI.cpp 
	g++ -std=c++11 -Wall -D_SPI -I$(SPIINC) -I$(MPUSRC) -c $(MPUSRC)/MPU9250_SPI.cpp

WiringPiSPI.o: $(SPISRC)/WiringPiSPI.cpp 
	g++ -std=c++11 -Wall -I$(SPIINC) -c $(SPISRC)/WiringPiSPI.cpp

main.o: ../../../main.cpp 
	g++ -std=c++11 -Wall -c ../../../main.cpp

run: MasterTest
	./MasterTest

edit:
	vim MasterTest.cpp

clean:
	rm -f $(ALL) *.o *~
