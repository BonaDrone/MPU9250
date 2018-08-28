/* MPU6050 Basic Example Code
by: Kris Winer
date: May 1, 2014
Modified August 2018 by Simon D. Levy
license: Beerware - Use this code however you'd like. If you 
find it useful you can buy me a beer some time.

Demonstrate  MPU-6050 basic functionality including initialization,
accelerometer trimming, sleep mode functionality as well as
parameterizing the register addresses. Added display functions to allow display to on breadboard monitor. 
No DMP use. We just want to get out the accelerations, temperature, and gyro readings.
 */

#include "MPU6050.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

static const Gscale_t GSCALE = GFS_250DPS;
static const Ascale_t ASCALE = AFS_2G;

static MPU6050 imu(ASCALE, GSCALE);;

static void error(const char * errmsg) 
{
    fprintf(stderr, "%s\n", errmsg);
    exit(1);
}

// defined in main.cpp
void delay(uint32_t msec);
uint32_t millis(void);

void setup()
{
    switch (imu.begin(0)) {

        case MPU_ERROR_IMU_ID:
            error("Bad device ID");
        case MPU_ERROR_SELFTEST:
            error("Failed self-test");
        default:
            printf("MPU6050 online!\n");
    }

    delay(1000);
}

void loop()
{  
    static float temperature;
    static uint32_t millisPrev;
    static float ax, ay, az;
    static float gx, gy, gz;

    // If data ready bit set, all data registers have new data
    if (imu.checkNewData()) {  // check if data ready interrupt

        imu.readAccelerometer(ax, ay, az);

        imu.readGyrometer(gx, gy, gz);

        temperature = imu.readTemperature();
    }  

    // Report data periodically
    if (millis()-millisPrev > 500) { 

        // Print acceleration values in milligs!
        printf("X-acceleration: %f mg ", 1000*ax);
        printf("Y-acceleration: %f mg ", 1000*ay);
        printf("Z-acceleration: %f mg\n", 1000*az);

        // Print gyro values in degree/sec
        printf("X-gyro rate: %4.1f degrees/sec  ", gx);
        printf("Y-gyro rate: %4.1f degrees/sec  ", gy);
        printf("Z-gyro rate: %4.1f degrees/sec\n", gz);

        // Print temperature in degrees Centigrade      
        printf("Temperature is %2.2f degrees C\n", temperature);

        millisPrev = millis();
    }

}
