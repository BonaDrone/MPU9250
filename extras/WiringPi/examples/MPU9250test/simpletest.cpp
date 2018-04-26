#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "MPU9250.h"
#include "WiringPiTransfer.h"

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
    uint8_t c = imu.getMPU9250ID();
    printf("MPU9250 I AM 0x%0X I should be 0x71\n", c);
    delay(1000);

    if (c != 0x71 ) { // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 

        fprintf(stderr, "Could not connect to MPU9250: 0x%02x\n", c);
        exit(1);
    }

    imu.resetMPU9250(); // start by resetting MPU9250

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    uint8_t d = imu.getAK8963CID();  // Read WHO_AM_I register for AK8963
    printf("AK8963 I AM 0x%0X   I should be 0x48\n", d);
    delay(1000); 

    return 0;
}

