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

MPUIMU::MPUIMU(Ascale_t ascale, Gscale_t gscale)
{
    _aRes = getAres(ascale);
    _gRes = getGres(gscale);

    _aScale = ascale;
    _gScale = gscale;
}

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

void MPUIMU::readAccelerometer(float & ax, float & ay, float & az)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here

    readMPURegisters(ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array

    int16_t x  = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    int16_t y  = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    int16_t z  = ((int16_t)rawData[4] << 8) | rawData[5] ; 

    // Convert the accleration value into g's
    ax = (float)x*_aRes - _accelBias[0];  
    ay = (float)y*_aRes - _accelBias[1];   
    az = (float)z*_aRes - _accelBias[2];  
}

void MPUIMU::readGyrometer(float & gx, float & gy, float & gz)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here

    readMPURegisters(GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array

    int16_t x = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
    int16_t y = ((int16_t)rawData[2] << 8) | rawData[3] ;  
    int16_t z = ((int16_t)rawData[4] << 8) | rawData[5] ; 

    // Convert the gyro value into degrees per second
    gx = (float)x*_gRes;  
    gy = (float)y*_gRes;  
    gz = (float)z*_gRes; 
}

int16_t MPUIMU::readRawTemperature(void)
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    readMPURegisters(TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
    return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}
