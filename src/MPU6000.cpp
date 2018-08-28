/*  Implementation of MPU6000 class library
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU6000.h"

#include "CrossPlatformSPI.h"

MPU6000::MPU6000(Ascale_t ascale, Gscale_t gscale) : MPU60x0(ascale, gscale)
{
}

MPU_Error_t MPU6000::begin(void)
{
    return MPU60x0::begin();
}

void MPU6000::writeMPURegister(uint8_t subAddress, uint8_t data)
{
    cpspi_writeRegister(subAddress, data);
}

void MPU6000::readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    cpspi_readRegisters(subAddress, count, dest);
}


