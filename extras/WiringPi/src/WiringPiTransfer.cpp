/*  Code for WiringPi implmentations of ByteTransfer class
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "WiringPiTransfer.h"

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>

void WiringPiI2C::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) 
{
    //wiringPiI2CWriteReg8(_i2c_fd, subAddress, data);

    //delay(10); // need to slow down how fast I write to WiringPiI2C
    //readRegisters(subAddress,sizeof(buff),&buff[0]);
    //return buff[0] == data;
}

uint8_t WiringPiI2C::readByte(uint8_t address, uint8_t subAddress) 
{
    return 0;//wiringPiI2CReadReg8(_i2c_fd, subAddress+i);
}

void WiringPiI2C::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    for (uint8_t i=0; i<count; ++i) {
        //dest[i] = wiringPiI2CReadReg8(_i2c_fd, subAddress+i);
    }
} 

WiringPiSPI::WiringPiSPI(uint8_t bus){
    _bus = bus;
}

void WiringPiSPI::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) 
{
    uint8_t buff2[2];
    buff2[0] = subAddress;
    buff2[1] = data;
    wiringPiSPIDataRW(_bus, &buff2[0], 2);

}

uint8_t WiringPiSPI::readByte(uint8_t address, uint8_t subAddress) 
{
    uint8_t buff2[2];

    buff2[0] = subAddress | 0x80;
    buff2[1] = 0;
    wiringPiSPIDataRW(_bus, &buff2[0], 2);
    return buff2[1];
}

void WiringPiSPI::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    uint8_t buff2[2];

    for (uint8_t i=0; i<count; ++i) {
        buff2[0] = (subAddress+i) | 0x80;
        buff2[1] = 0;
        wiringPiSPIDataRW(_bus, &buff2[0], 2);
        dest[i] = buff2[1];
    }
} 
