/* MPU6050 Basic Example Code
by: Kris Winer
date: May 1, 2014
Modified August 2018 by Simon D. Levy
license: Beerware - Use this code however you'd like. If you 
find it useful you can buy me a beer some time.

Demonstrate  MPU-6050 basic functionality including initialization, accelerometer trimming, sleep mode functionality as well as
parameterizing the register addresses. Added display functions to allow display to on breadboard monitor. 
No DMP use. We just want to get out the accelerations, temperature, and gyro readings.

SDA and SCL should have external pull-up resistors (to 3.3V).
10k resistors worked for me. They should be on the breakout
board.

Hardware setup:
MPU6050 Breakout --------- Arduino
3.3V --------------------- 3.3V
SDA ----------------------- A4
SCL ----------------------- A5
GND ---------------------- GND

Note: The MPU6050 is an I2C sensor and uses the Arduino Wire library. 
Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */

#include "MPU6050.h"

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

static const Ascale_t ASCALE = AFS_2G;
static const Gscale_t GSCALE = GFS_250DPS;

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

        case MPU_ERROR_IMU_ID:
            error("Bad device ID");
        case MPU_ERROR_SELFTEST:
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
