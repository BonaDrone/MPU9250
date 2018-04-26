/* 07/6/2017 Copyright Tlera Corporation
 *  
 * Created by Kris Winer
 *
 * Adapted by Simon D. Levy April 2018
 *  
 * Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 * getting properly scaled accelerometer, gyroscope, and magnetometer data out. 
 * Computes quaternion and roll/pitch/yaw using Madgwick's algorithm.
 *
 * Library may be used freely and without limit with attribution.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "MPU9250.h"
#include "WiringPiTransfer.h"
#include "QuaternionFilters.h"

/*
 MPU9250 Configuration

 Specify sensor full scale

 Choices are:

  Gscale:   GFS_250 = 250 dps, GFS_500 = 500 dps, GFS_1000 = 1000 dps, GFS_2000DPS = 2000 degrees per second gyro full scale

  Ascale: AFS_2G = 2 g, AFS_4G = 4 g, AFS_8G = 8 g, and AFS_16G = 16 g accelerometer full scale

  Mscale: MFS_14BITS = 0.6 mG per LSB and MFS_16BITS = 0.15 mG per LSB

  Mmode: Mmode = M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate

  sampleRate: (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
  sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
*/
static const uint8_t Gscale     = GFS_250DPS;
static const uint8_t Ascale     = AFS_2G;
static const uint8_t Mscale     = MFS_16BITS;
static const uint8_t Mmode      = M_100Hz;
static const uint8_t sampleRate = 0x04;         

// scale resolutions per LSB for the sensors
//static float aRes, gRes, mRes;

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
static const float GyroMeasError = M_PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
static const float GyroMeasDrift = M_PI * (0.0f  / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
static const float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta


// SPI settings
static const uint8_t SPI_BUS = 1;
static const uint32_t SPI_SPEED = 400000;

int main(int argc, char ** argv)
{
    // Set up SPI transfer object
    WiringPiSPI bt = WiringPiSPI(SPI_BUS, SPI_SPEED);

    // Instantiate MPU9250 class
    MPU9250 imu = MPU9250(&bt); 

    // Begin SPI
    if (wiringPiSPISetup(SPI_BUS, SPI_SPEED) < 0) {
        fprintf(stderr, "Unable to start SPI\n");
        exit(1);
    }

    // Read the WHO_AM_I register, this is a good test of communication
    printf("MPU9250 9-axis motion sensor...\n");
    uint8_t c = imu.getMPU9250ID();
    printf("MPU9250 I AM 0x%0X I should be 0x71\n", c);
    delay(1000);

    return 0;
}

