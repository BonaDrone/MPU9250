/*  Implementation of MPU6500 class library
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU6500.h"

#include <CrossPlatformSPI.h>

MPU6500::MPU6500(Ascale_t ascale, Gscale_t gscale) : MPUIMU(ascale, gscale)
{
}

MPU_Error_t MPU6500::begin(void)
{
    cpspi_writeRegister(PWR_MGMT_1, 0x80);
    delay(100);
    cpspi_writeRegister(SIGNAL_PATH_RESET, 0x80);
    delay(100);
    cpspi_writeRegister(PWR_MGMT_1, 0);
    delay(100);
    cpspi_writeRegister(PWR_MGMT_1, INV_CLK_PLL);
    delay(15);
    cpspi_writeRegister(GYRO_CONFIG, 0x00);//_gScale << 3);
    delay(15);
    cpspi_writeRegister(ACCEL_CONFIG, 0x00);//_aScale << 3);
    delay(15);
    cpspi_writeRegister(CONFIG, 0); // no DLPF bits
    delay(15);
    cpspi_writeRegister(SMPLRT_DIV, 0); 
    delay(100);

    // Data ready interrupt configuration
    cpspi_writeRegister(INT_PIN_CFG, 0x10);  
    delay(15);

    cpspi_writeRegister(INT_ENABLE, 0x01); 
    delay(15);

    _accelBias[0] = 0;
    _accelBias[1] = 0;
    _accelBias[2] = 0;
    
    return MPU_ERROR_NONE;
}

bool MPU6500::checkNewData(void)
{
    uint8_t data;
    cpspi_readRegisters(INT_STATUS | 0x80, 1, &data);
    return (bool)(data & 0x01);
}

void MPU6500::readGyrometer(float & gx, float & gy, float & gz)
{
    MPUIMU::readGyrometer(gx, gy, gz);
}

void MPU6500::writeMPURegister(uint8_t subAddress, uint8_t data) 
{
    cpspi_writeRegister(subAddress, data);
}

void MPU6500::readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) 
{
    cpspi_readRegisters(subAddress | 0x80, count, dest);
}
