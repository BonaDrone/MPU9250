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
static float aRes, gRes, mRes;

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
static const float GyroMeasError = M_PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
static const float GyroMeasDrift = M_PI * (0.0f  / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
static const float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta

// SPI settings
static const uint8_t SPI_BUS = 1;
static const uint32_t SPI_SPEED = 400000;

int main(int argc, char ** argv)
{
    // Bias corrections for gyro and accelerometer. These can be measured once and
    // entered here or can be calculated each time the device is powered on.
    float gyroBias[3], accelBias[3], magBias[3], magScale[3];      

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

    if (c == 0x71 ) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
    {  
        printf("MPU9250 is online...\n");

        imu.resetMPU9250(); // start by resetting MPU9250

        float SelfTest[6];    // holds results of gyro and accelerometer self test

        imu.SelfTest(SelfTest); // Start by performing self test and reporting values

        printf("x-axis self test: acceleration trim within %2.2f%% of factory value\n", SelfTest[0]);
        printf("y-axis self test: acceleration trim within %2.2f%% of factory value\n", SelfTest[1]);
        printf("z-axis self test: acceleration trim within %2.2f%% of factory value\n", SelfTest[2]);

        printf("x-axis self test: gyration trim within %2.2f%% of factory value\n", SelfTest[3]);
        printf("y-axis self test: gyration trim within %2.2f%% of factory value\n", SelfTest[4]);
        printf("z-axis self test: gyration trim within %2.2f%% of factory value\n", SelfTest[5]);

        delay(1000);

        // get sensor resolutions, only need to do this once
        aRes = imu.getAres(Ascale);
        gRes = imu.getGres(Gscale);
        mRes = imu.getMres(Mscale);

        // Comment out if using pre-measured, pre-stored offset biases
        imu.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
        printf("accel biases (mg)\n");
        printf("%f\n", 1000.*accelBias[0]);
        printf("%f\n", 1000.*accelBias[1]);
        printf("%f\n", 1000.*accelBias[2]);
        printf("gyro biases (dps)\n");
        printf("%f\n", gyroBias[0]);
        printf("%f\n", gyroBias[1]);
        printf("%f\n", gyroBias[2]);
        delay(1000); 

        imu.initMPU9250(Ascale, Gscale, sampleRate); 
        printf("MPU9250 initialized for active data mode....\n"); 

        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        uint8_t d = imu.getAK8963CID();  // Read WHO_AM_I register for AK8963
        printf("AK8963 I AM 0x%02x   I should be 0x48\n", d);
        delay(1000); 

        /*
        // Get magnetometer calibration from AK8963 ROM
        imu.initAK8963(Mscale, Mmode, magCalibration);
        printf("AK8963 initialized for active data mode...."); 

        // Comment out if using pre-measured, pre-stored offset biases
        printf("Mag Calibration: Wave device in a figure eight until done!");
        delay(4000);
        imu.magcalMPU9250(magBias, magScale);
        printf("Mag Calibration done!");
        printf("AK8963 mag biases (mG)");
        printf(magBias[0]);
        printf(magBias[1]);
        printf(magBias[2]); 
        printf("AK8963 mag scale (mG)");
        printf(magScale[0]);
        printf(magScale[1]);
        printf(magScale[2]); 
        delay(2000); // add delay to see results before serial spew of data

        printf("Calibration values: ");
        printf("X-Axis sensitivity adjustment value ");
        printf(magCalibration[0], 2);
        printf("Y-Axis sensitivity adjustment value ");
        printf(magCalibration[1], 2);
        printf("Z-Axis sensitivity adjustment value ");
        printf(magCalibration[2], 2);

        attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for intPin output of MPU9250
        */

    }
    else
    {
        fprintf(stderr, "Could not connect to MPU9250: 0x%02x\n", c);
        exit(1);
    }


    return 0;
}

