/*  MPU9250 magnetometer calibration
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. Levy 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include <Wire.h>   

#include "ArduinoTransfer.h"
#include "MPU9250.h"

// Define I2C addresses of MPU9250
#define ADO 0
#if ADO
static const uint8_t MPU9250_ADDRESS  = 0x69;   // Device address when ADO = 1
#else
static const uint8_t MPU9250_ADDRESS  = 0x68;  // Device address when ADO = 0
#endif  

// Create a byte-transfer object for Arduino I^2C
ArduinoI2C bt;

// Instantiate MPU9250 class in pass-through mode
static MPU9250Passthru imu = MPU9250Passthru(&bt); 

void setup(void)
{
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

    if(_Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(_Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
    for(ii = 0; ii < sample_count; ii++) {
        imu.readMagData(mag_temp);  // Read the mag data   
        for (int jj = 0; jj < 3; jj++) {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        if(_Mmode == 0x02) _bt->delayMsec(135);  // at 8 Hz ODR, new mag data is available every 125 ms
        if(_Mmode == 0x06) _bt->delayMsec(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*_mRes*_magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*_mRes*_magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*_mRes*_magCalibration[2];  

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
}

void loop(void)
{
}


