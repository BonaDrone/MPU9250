/*  Code for WiringPi implmentations of ByteTransfer class
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "WiringPiTransfer.h"

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <stdio.h>
#include <stdlib.h>

void WiringPiI2C::begin(void)
{
    _fd = wiringPiI2CSetup (_address);
}

void WiringPiI2C::writeRegister(uint8_t subAddress, uint8_t data)
{
	wiringPiI2CWriteReg8(_fd, subAddress, data);
}

uint8_t WiringPiI2C::readRegister(uint8_t subAddress)
{
    uint8_t data = 0;
    readRegisters(subAddress, 1, &data);
    return data;                             
}

void WiringPiI2C::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    for (uint8_t i=0; i<count; ++i) {
        dest[i] = wiringPiI2CReadReg8(_fd, subAddress+i);
    }
}




