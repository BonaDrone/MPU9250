/*  Implementation code for MPU9250 class library
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. Levy 19 April 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU9250_Passthru.h"

#include <CrossPlatformI2C.h>

MPU9250_Passthru::MPU9250_Passthru(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor) :
    MPU9250(ascale, gscale, mscale, mmode, sampleRateDivisor, true)
{
}

MPU_Error_t MPU9250_Passthru::begin(uint8_t i2cbus)
{
    MPU9250::begin(i2cbus);

    _mag = cpi2c_open(AK8963_ADDRESS);

    return runTests();
}
bool MPU9250_Passthru::checkNewAccelGyroData()
{
    return MPUIMU::checkNewData();
}

bool MPU9250_Passthru::checkNewMagData()
{
    return readAK8963Register(AK8963_ST1) & 0x01;
}

void MPU9250_Passthru::writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    writeRegister(_mag, subAddress, data);
}

void MPU9250_Passthru::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    readRegisters(_mag, subAddress, count, dest);
}
