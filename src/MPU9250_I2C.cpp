/* 
   MPU9250_I2C.cpp: I^2C support for MPU9250 class

   Copyright (C) 2018 Simon D. Levy

   This file is part of MPU.

   MPU is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   MPU is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with MPU.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "MPU9250.h"
#include "CrossPlatformI2C.h"

#include <math.h>

// One ifdef needed to support delay() cross-platform
#if defined(ARDUINO)
#include <Arduino.h>
#elif defined(__arm__)
#include <wiringPi.h>
#else
extern void delay(uint32_t msec);
#endif


MPU_Error_t MPU9250::begin(uint8_t bus)
{
    _mpu = cpi2c_open(MPU_ADDRESS, bus);

    return runTests();
}

uint8_t MPU9250::readRegister(uint8_t address, uint8_t subAddress)
{
    uint8_t data = 0;
    cpi2c_readRegisters(address, subAddress, 1, &data);
    return data;
}

void MPU9250::readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * data)
{
    cpi2c_readRegisters(address, subAddress, count, data);
}


void MPU9250::writeRegister(uint8_t address, uint8_t subAddress, uint8_t data)
{
    cpi2c_writeRegister(address, subAddress, data);
}



