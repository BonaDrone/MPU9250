/*  Implementation of MPU6500 class library
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU6500.h"

MPU6500::MPU6500(Ascale_t ascale, Gscale_t gscale) 
{
    (void)ascale;
    (void)gscale;
}

MPU_Error_t MPU6500::begin(void)
{
    return MPU_ERROR_NONE;
}

bool MPU6500::checkNewData(void)
{
    return false;
}

void MPU6500::readAccelerometer(float & ax, float & ay, float & az)
{
    ax = 0;
    ay = 0;
    az = 0;
}

void MPU6500::readGyrometer(float & gx, float & gy, float & gz)
{
    gx = 0;
    gy = 0;
    gz = 0;
}

void MPU6500::writeMPURegister(uint8_t subAddress, uint8_t data) 
{
    (void)subAddress;
    (void)data;
}

void MPU6500::readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) 
{
    (void)subAddress;
    (void)count;
    (void)dest;
}

uint8_t MPU6500::xAOffsetH(void) 
{
    return 0;
}

uint8_t MPU6500::yAOffsetH(void) 
{
    return 0;
}

uint8_t MPU6500::zAOffsetH(void) 
{
    return 0;
}
