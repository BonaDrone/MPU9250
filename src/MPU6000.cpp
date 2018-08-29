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
   writeMPURegister(PWR_MGMT_1, 0x80);
    delay(100);
    writeMPURegister(SIGNAL_PATH_RESET, 0x80);
    delay(100);
    writeMPURegister(PWR_MGMT_1, 0);
    delay(100);
    writeMPURegister(PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    writeMPURegister(GYRO_CONFIG, 0x00);//_gScale << 3);
    delay(15);
    writeMPURegister(ACCEL_CONFIG, 0x00);//_aScale << 3);
    delay(15);
    writeMPURegister(CONFIG, 0); // no DLPF bits
    delay(15);
    writeMPURegister(SMPLRT_DIV, 0); 
    delay(100);

    // Data ready interrupt configuration
    writeMPURegister(INT_PIN_CFG, 0x10);  
    delay(15);

    writeMPURegister(INT_ENABLE, 0x01); 
    delay(15);

    _accelBias[0] = 0;
    _accelBias[1] = 0;
    _accelBias[2] = 0;
    
    return MPU_ERROR_NONE;
}

void MPU6000::writeMPURegister(uint8_t subAddress, uint8_t data)
{
    cpspi_writeRegister(subAddress, data);
}

void MPU6000::readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    cpspi_readRegisters(subAddress, count, dest);
}


