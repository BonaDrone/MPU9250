#   Makefile for MPU9250 WiringPi examples
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
CPDB = $(HOME)/CrossPlatformDataBus

ALL = MasterTest PassthruTest SPITest

MPUSRC = ../../../../src

CPDBINC = $(CPDB)/src
I2CSRC = $(CPDB)/extras/i2c/wiringpi/src
SPISRC = $(CPDB)/extras/spi/wiringpi/src

all: $(ALL)

MasterTest: MasterTest.o MPU.o MPU9250.o MPU9250_Master.o MPU9250_Master_I2C.o WiringPiI2C.o main.o
	g++ -std=c++11 -o MasterTest MasterTest.o MPU.o MPU9250.o MPU9250_Master.o MPU9250_Master_I2C.o WiringPiI2C.o main.o -lwiringPi

MasterTest.o: MasterTest.cpp 
	g++ -std=c++11 -Wall -I$(CPDBINC) -I$(MPUSRC) -c MasterTest.cpp

PassthruTest: PassthruTest.o MPU.o MPU9250.o MPU9250_Passthru.o WiringPiI2C.o main.o
	g++ -std=c++11 -o PassthruTest PassthruTest.o MPU.o MPU9250.o MPU9250_Passthru.o WiringPiI2C.o main.o -lwiringPi

PassthruTest.o: PassthruTest.cpp 
	g++ -std=c++11 -Wall -I$(CPDBINC) -I$(MPUSRC) -c PassthruTest.cpp

SPITest: SPITest.o MPU.o MPU9250.o MPU9250_Master.o MPU9250_Master_SPI.o WiringPiSPI.o main.o
	g++ -std=c++11 -o SPITest SPITest.o MPU.o MPU9250.o MPU9250_Master.o MPU9250_Master_SPI.o WiringPiSPI.o main.o -lwiringPi

SPITest.o: SPITest.cpp 
	g++ -std=c++11 -Wall -I$(CPDBINC) -I$(MPUSRC) -c SPITest.cpp

MPU.o: $(MPUSRC)/MPU.cpp 
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU.cpp

MPU9250.o: $(MPUSRC)/MPU9250.cpp 
	g++ -std=c++11 -Wall -I$(MPUSRC) -c $(MPUSRC)/MPU9250.cpp

MPU9250_Master_I2C.o: $(MPUSRC)/MPU9250_Master_I2C.cpp 
	g++ -std=c++11 -Wall -I$(CPDBINC) -I$(MPUSRC) -c $(MPUSRC)/MPU9250_Master_I2C.cpp

MPU9250_Master.o: $(MPUSRC)/MPU9250_Master.cpp 
	g++ -std=c++11 -Wall -I$(CPDBINC) -I$(MPUSRC) -c $(MPUSRC)/MPU9250_Master.cpp

MPU9250_Passthru.o: $(MPUSRC)/MPU9250_Passthru.cpp 
	g++ -std=c++11 -Wall -I$(CPDBINC) -I$(MPUSRC) -c $(MPUSRC)/MPU9250_Passthru.cpp

MPU9250_Master_SPI.o: $(MPUSRC)/MPU9250_Master_SPI.cpp 
	g++ -std=c++11 -Wall -D_SPI -I$(CPDBINC) -I$(MPUSRC) -c $(MPUSRC)/MPU9250_Master_SPI.cpp

WiringPiSPI.o: $(SPISRC)/WiringPiSPI.cpp 
	g++ -std=c++11 -Wall -I$(CPDBINC) -c $(SPISRC)/WiringPiSPI.cpp

WiringPiI2C.o: $(I2CSRC)/WiringPiI2C.cpp 
	g++ -std=c++11 -Wall -I$(CPDBINC) -c $(I2CSRC)/WiringPiI2C.cpp

main.o: ../../main.cpp 
	g++ -std=c++11 -Wall -c ../../main.cpp

run: MasterTest
	./MasterTest

edit:
	vim MasterTest.cpp

clean:
	rm -f $(ALL) *.o *~
