/* 
   MPU9250_Master_I2C.cpp: I^2C support for MPU9250 master class

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

#include "MPU9250_Master_I2C.h"

#include <CrossPlatformI2C.h>

// One ifdef needed to support delay() cross-platform
#if defined(ARDUINO)
#include <Arduino.h>
#elif defined(__arm__)
#include <wiringPi.h>
#else
extern void delay(uint32_t msec);
#endif

MPU9250_Master_I2C::MPU9250_Master_I2C(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor) :
    MPU9250_Master(ascale, gscale, mscale, mmode, sampleRateDivisor)
{
}

MPUIMU::Error_t MPU9250_Master_I2C::begin(uint8_t bus)
{
    _i2c = cpi2c_open(MPU_ADDRESS, bus);

    return runTests();
}

void MPU9250_Master_I2C::readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * data)
{
    cpi2c_readRegisters(address, subAddress, count, data);
}


void MPU9250_Master_I2C::writeRegister(uint8_t address, uint8_t subAddress, uint8_t data)
{
    cpi2c_writeRegister(address, subAddress, data);
}

void MPU9250_Master_I2C::writeMPURegister(uint8_t subAddress, uint8_t data)
{
    cpi2c_writeRegister(_i2c, subAddress, data);
}

void MPU9250_Master_I2C::readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    cpi2c_readRegisters(_i2c, subAddress, count, dest);
}
