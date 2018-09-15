/* 
   PassthruTest.cpp: example of how to use MPU9250 in pass-through mode

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

#include <MPU9250_Passthru.h>
#include <wiringPi.h>
#include <stdio.h>
#include <errmsg.h>

/*
   MPU9250 Configuration

   Specify sensor full scale

   Choices are:

Gscale: GFS_250 = 250 dps, GFS_500 = 500 dps, GFS_1000 = 1000 dps, GFS_2000DPS = 2000 degrees per second gyro full scale
Ascale: AFS_2G = 2 g, AFS_4G = 4 g, AFS_8G = 8 g, and AFS_16G = 16 g accelerometer full scale
Mscale: MFS_14BITS = 0.6 mG per LSB and MFS_16BITS = 0.15 mG per LSB
Mmode:  Mmode = M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
sampleRate: (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
 */

static const MPUIMU::Gscale_t  GSCALE     = MPUIMU::GFS_250DPS;
static const MPUIMU::Ascale_t  ASCALE     = MPUIMU::AFS_2G;
static const MPU9250::Mscale_t MSCALE    = MPU9250::MFS_16BITS;
static const MPU9250::Mmode_t  MMODE     = MPU9250::M_100Hz;
static const uint8_t SAMPLE_RATE_DIVISOR = 0x04;         

// Pin definitions
static const uint8_t INTERRUPT_PIN = 0;   //  MPU9250 interrupt

// Instantiate MPU9250 class in pass-through mode
static MPU9250_Passthru imu(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);

void setup()
{
    // Setup WirinPi
    wiringPiSetup();

    delay(100);

    // Start the MPU9250
    switch (imu.begin()) {

        case MPUIMU::ERROR_IMU_ID:
            errmsg("Bad IMU device ID");
            break;
        case MPUIMU::ERROR_MAG_ID:
            errmsg("Bad magnetometer device ID");
            break;
        case MPUIMU::ERROR_SELFTEST:
            //errmsg("Failed self-test");
            break;
        default:
            printf("MPU6050 online!\n");
            break;
    }

    // Comment out if using pre-measured, pre-stored offset magnetometer biases
    //printf("Mag Calibration: Wave device in a figure eight until done!\n");
    //imu.calibrateMagnetometer();
}

void loop()
{  
    static float ax, ay, az, gx, gy, gz, mx, my, mz, temperature;

    if (imu.checkNewAccelGyroData())  { 

        imu.readAccelerometer(ax, ay, az);

        imu.readGyrometer(gx, gy, gz);

        temperature = imu.readTemperature();

        if(imu.checkNewMagData()) { 

            imu.readMagnetometer(mx, my, mz);
        }
    }

    // Report at 4 Hz
    static uint32_t msec_prev;
    uint32_t msec_curr = millis();

    if (msec_curr-msec_prev > 250) {

        msec_prev = msec_curr;

        printf("\n");

        printf("ax = %d  ay = %d  az = %d mg\n", (int)(1000*ax), (int)(1000*ay), (int)(1000*az));
        printf("gx = %+2.2f  gy = %+2.2f  gz = %+2.2f deg/s\n", gx, gy, gz);
        printf("mx = %d  my = %d  mz = %d mG\n", (int)mx, (int)my, (int)mz);

        // Print temperature in degrees Centigrade      
        printf("Gyro temperature is %+1.1f degrees C\n", temperature);  
    }

}
