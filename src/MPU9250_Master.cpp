/*  Implementation code for MPU9250 master-mode class 
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. Levy 19 April 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU9250_Master.h"

MPU9250_Master::MPU9250_Master(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor) :
    MPU9250(ascale, gscale, mscale, mmode, sampleRateDivisor, false)
{
}

MPU_Error_t MPU9250_Master::begin(uint8_t i2cbus)
{
    MPU9250::begin(i2cbus);

    return runTests();
}

void MPU9250_Master::initMPU6500(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor) 
{ 
    MPU9250::initMPU6500(ascale, gscale, sampleRateDivisor, false); 
}

void MPU9250_Master::writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    uint8_t count = 1;

    writeMPURegister(I2C_SLV0_ADDR, AK8963_ADDRESS); // set slave 0 to the AK8963 and set for write
    writeMPURegister(I2C_SLV0_REG, subAddress); // set the register to the desired AK8963 sub address
    writeMPURegister(I2C_SLV0_DO, data); // store the data for write
    writeMPURegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and send 1 byte
}

void MPU9250_Master::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    writeMPURegister(I2C_SLV0_ADDR, AK8963_ADDRESS | I2C_READ_FLAG); // set slave 0 to the AK8963 and set for read
    writeMPURegister(I2C_SLV0_REG, subAddress); // set the register to the desired AK8963 sub address
    writeMPURegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and request the bytes
    delay(1); // takes some time for these registers to fill
    readMPURegisters(EXT_SENS_DATA_00, count, dest); // read the bytes off the MPU9250 EXT_SENS_DATA registers
}

bool MPU9250_Master::checkNewData(void)
{
    return (readMPURegister(INT_STATUS) & 0x01);
}
