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
    _fd = wiringPiI2CSetup (_i2c_address);
}

void WiringPiI2C::writeRegister(uint8_t subAddress, uint8_t data)
{
}

uint8_t WiringPiI2C::readRegister(uint8_t subAddress)
{
    return data;                             // Return data read from slave register
}

void WiringPiI2C::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(_address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(_address, count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read();          // Put read results in the Rx buffer
    }
}




