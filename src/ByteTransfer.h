/*  Header file for abstract ByteTransfer class supporting I^2C and SPI
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#pragma once

#include <stdint.h>

class ByteTransfer
{
    public: 

        virtual void    delayMsec(unsigned long msec) = 0;
        virtual void    writeRegister(uint8_t address, uint8_t subAddress, uint8_t data) = 0;
        virtual uint8_t readRegister(uint8_t address, uint8_t subAddress) = 0;
        virtual void    readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) = 0;
};

class I2CTransfer
{
};
