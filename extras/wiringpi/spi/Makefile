# 
# Makefile for MPU9250 WiringPi SPI example
#
# Copyright (c) 2018 Simon D. Levy
#
# This file is part of MPU9250.
#
# Library may be used freely and without limit with attribution.
  
# Change this to whereever you installed CrossPlatformSPI
CPSPI = $(HOME)/CrossPlatformSPI/

ALL = SPITest

MPUSRC = ../../../src

SPIINC = $(CPSPI)/src
SPISRC = $(CPSPI)/extras/wiringpi/src

all: $(ALL)

SPITest: SPITest.o MPU9250.o MPU9250_SPI.o WiringPiSPI.o main.o
	g++ -std=c++11 -o SPITest SPITest.o MPU9250.o MPU9250_SPI.o WiringPiSPI.o main.o -lwiringPi

SPITest.o: SPITest.cpp 
	g++ -std=c++11 -Wall -I$(SPIINC) -I$(MPUSRC) -c SPITest.cpp

MPU9250.o: $(MPUSRC)/MPU9250.cpp 
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU9250.cpp

MPU9250_SPI.o: $(MPUSRC)/MPU9250_SPI.cpp 
	g++ -std=c++11 -Wall -D_SPI -I$(SPIINC) -I$(MPUSRC) -c $(MPUSRC)/MPU9250_SPI.cpp

WiringPiSPI.o: $(SPISRC)/WiringPiSPI.cpp 
	g++ -std=c++11 -Wall -I$(SPIINC) -c $(SPISRC)/WiringPiSPI.cpp

main.o: ../../main.cpp 
	g++ -std=c++11 -Wall -c ../../main.cpp

run: MasterTest
	./MasterTest

edit:
	vim MasterTest.cpp

clean:
	rm -f $(ALL) *.o *~
