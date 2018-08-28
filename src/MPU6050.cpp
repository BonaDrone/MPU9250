/*  Implementation of MPU6050 class library
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU6050.h"

#include "CrossPlatformI2C_Core.h"

MPU6050::MPU6050(Ascale_t ascale, Gscale_t gscale) : MPU60x0(ascale, gscale)
{
}

MPU_Error_t MPU6050::begin(uint8_t bus)
{
    _i2c = cpi2c_open(MPU60x0_ADDRESS, bus);

    return MPU60x0::begin();
}

void MPU6050::writeMPURegister(uint8_t subAddress, uint8_t data)
{
    cpi2c_writeRegister(_i2c, subAddress, data);
}

void MPU6050::readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    cpi2c_readRegisters(_i2c, subAddress, count, dest);
}


