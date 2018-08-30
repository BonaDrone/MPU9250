  /* 
   MPU6000.cpp: Implementation of MPU6000 classes

   Copyright (C) 2018 Simon D. Levy

   This file is part of MPU.

   MPU is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   MPU is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with MPU.  If not, see <http://www.gnu.org/licenses/>.
   */

#include "MPU6000.h"

#include "CrossPlatformSPI.h"

MPU6000::MPU6000(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor) : 
    MPU6xx0(ascale, gscale, sampleRateDivisor)
{
}

MPU_Error_t MPU6000::begin(void)
{
    if (getId() != MPU_ADDRESS) {
        return MPU_ERROR_IMU_ID;
    }

    if (!selfTest()) {
        return MPU_ERROR_SELFTEST;
    }

    writeMPURegister(PWR_MGMT_1, 0x80);
    delay(100);

    writeMPURegister(SIGNAL_PATH_RESET, 0x80);
    delay(100);

    writeMPURegister(PWR_MGMT_1, 0x00);
    delay(100);
    writeMPURegister(PWR_MGMT_1, INV_CLK_PLL);
    delay(15);

    writeMPURegister(GYRO_CONFIG, _gScale << 3);
    delay(15);
    writeMPURegister(ACCEL_CONFIG, _aScale << 3);
    delay(15);

    writeMPURegister(CONFIG, 0); // no DLPF bits
    delay(15);

    writeMPURegister(SMPLRT_DIV, _sampleRateDivisor); 
    delay(100);

    _accelBias[0] = 0;
    _accelBias[1] = 0;
    _accelBias[2] = 0;

    //calibrate();

    // Data ready interrupt configuration
    writeMPURegister(INT_PIN_CFG, 0x10);  
    delay(15);

    writeMPURegister(INT_ENABLE, 0x01); 
    delay(15);

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
