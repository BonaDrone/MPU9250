/*  I2C implementation code for MPU9250 class library
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. Levy 19 April 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU9250.h"
#include "CrossPlatformI2C.h"

#include <math.h>

// One ifdef needed to support delay() cross-platform
#if defined(ARDUINO)
#include <Arduino.h>
#elif defined(__arm__)
#include <wiringPi.h>
#else
extern void delay(uint32_t msec);
#endif


void MPU9250::begin(void)
{
    _mpu = cpi2c_open(MPU9250_ADDRESS);
}

uint8_t MPU9250::readRegister(uint8_t address, uint8_t subAddress)
{
    uint8_t data = 0;
    cpi2c_readRegisters(address, subAddress, 1, &data);
    return data;
}

void MPU9250::readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * data)
{
    cpi2c_readRegisters(address, subAddress, count, data);
}


void MPU9250::writeRegister(uint8_t address, uint8_t subAddress, uint8_t data)
{
    cpi2c_writeRegister(address, subAddress, data);
}

// Passthru ===========================================================================================

void MPU9250_Passthru::begin(void)
{
    MPU9250::begin();
    _mag = cpi2c_open(AK8963_ADDRESS);
}
