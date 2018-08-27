/*  Common methods for MPU classes
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU.h"

float MPUIMU::getGres(Gscale_t gscale) 
{
    switch (gscale) {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
            return 250.0 / 32768.0;
            break;
        case GFS_500DPS:
            return 500.0 / 32768.0;
            break;
        case GFS_1000DPS:
            return 1000.0 / 32768.0;
            break;
        case GFS_2000DPS:
            return 2000.0 / 32768.0;
            break;
    }

    return 0;
}

float MPUIMU::getAres(Ascale_t ascale) 
{
    switch (ascale) {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
            return 2.0 / 32768.0;
            break;
        case AFS_4G:
            return 4.0 / 32768.0;
            break;
        case AFS_8G:
            return 8.0 / 32768.0;
            break;
        case AFS_16G:
            return 16.0 / 32768.0;
            break;
    }

    return 0;
}

uint8_t MPUIMU::getId()
{
    return readMPURegister(WHO_AM_I);  // Read WHO_AM_I register for MPU-9250
}

uint8_t MPUIMU::readMPURegister(uint8_t subAddress)
{
    uint8_t data;
    readMPURegisters(subAddress, 1, &data);
    return data;
}
