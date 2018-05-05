/*  Code for WiringPi implmentations of ByteTransfer class
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "WiringPiTransfer.h"

#include <wiringPi.h>
#include <wiringPiI2C.h>

void WiringPiI2C::begin(void)
{
    _fd = wiringPiI2CSetup (_address);
}

void WiringPiI2C::writeRegister(uint8_t subAddress, uint8_t data)
{
}

uint8_t WiringPiI2C::readRegister(uint8_t subAddress)
{
    uint8_t data = 0;

    return data;                             // Return data read from slave register
}

void WiringPiI2C::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
}




