/* 
   Basic.ino: MPU6050 basic example

   Copyright (C) 2018 Simon D. Levy

   Adapted from https://github.com/kriswiner/MPU6050/blob/master/MPU6050BasicExample.ino

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

#include "MPU6050.h"

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

static const MPUIMU::Ascale_t ASCALE = MPUIMU::AFS_2G;
static const MPUIMU::Gscale_t GSCALE = MPUIMU::GFS_250DPS;

static MPU6050 imu(ASCALE, GSCALE);;

static void error(const char * errmsg) 
{
    Serial.println(errmsg);
    while (true) ;
}

static void reportAcceleration(const char * dim, float val)
{
    Serial.print(dim);
    Serial.print("-acceleration: ");
    Serial.print(1000*val);
    Serial.print(" mg "); 
}

static void reportGyroRate(const char * dim, float val)
{
    Serial.print(dim);
    Serial.print("-gyro rate: ");
    Serial.print(val, 1);
    Serial.print(" degrees/sec "); 
}

void setup()
{
    Serial.begin(115200);

#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    switch (imu.begin()) {

        case MPUIMU::ERROR_IMU_ID:
            error("Bad device ID");
        case MPUIMU::ERROR_SELFTEST:
            error("Failed self-test");
        default:
            Serial.println("MPU6050 online!\n");
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
    if ((millis() - millisPrev) > 500) {

        reportAcceleration("X", ax);
        reportAcceleration("Y", ay);
        reportAcceleration("Z", az);

        Serial.println();

        reportGyroRate("X", ax);
        reportGyroRate("Y", ay);
        reportGyroRate("Z", az);

        Serial.println();

        // Print temperature in degrees Centigrade      
        Serial.print("Temperature is ");  
        Serial.print(temperature, 2);  
        Serial.println(" degrees C"); // Print T values to tenths of s degree C
        Serial.println("");

        millisPrev = millis();
    }
}
