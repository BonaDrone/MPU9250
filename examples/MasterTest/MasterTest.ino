/* 07/6/2017 Copyright Tlera Corporation
 *  
 * Created by Kris Winer
 *
 * Adapted by Simon D. Levy April 2018
 *  
 * Demonstrate basic MPU-9250 functionality in master mode including
 * parameterizing the register addresses, initializing the sensor, getting
 * properly scaled accelerometer, gyroscope, and magnetometer data out. 
 *
 * SDA and SCL have 4K7 pull-up resistors (to 3.3V).
 *
 * Library may be used freely and without limit with attribution.
 */


#include <Wire.h>

#include "MPU9250.h"
#include "ArduinoTransfer.h"

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
static const uint8_t Gscale     = GFS_250DPS;
static const uint8_t Ascale     = AFS_2G;
static const uint8_t Mscale     = MFS_16BITS;
static const uint8_t Mmode      = M_100Hz;
static const uint8_t sampleRate = 0x04;         

// scale resolutions per LSB for the sensors
static float aRes, gRes, mRes;

// Pin definitions
static const uint8_t intPin = 8;   //  MPU9250 interrupt
static const uint8_t ledPin = 13; // red led

// Interrupt support 
static bool gotNewData = false;
static void myinthandler()
{
    gotNewData = true;
}

// Create a byte-transfer object for Arduino I^2C
ArduinoI2C bt;

// Factory mag calibration and mag bias
static float   magCalibration[3]; 

// Instantiate MPU9250 class in master mode
static MPU9250 imu = MPU9250(&bt, false); 

void setup(void)
{
    Serial.begin(115200);
    delay(1000);

    Wire.begin();
    Wire.setClock(400000);
    delay(1000);

    // Set up the interrupt pin, it's set as active high, push-pull
    pinMode(intPin, INPUT);

    // Start with orange led on (active HIGH)
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH); 

    // Configure the MPU9250 
    // Read the WHO_AM_I register, this is a good test of communication
    Serial.println("MPU9250 9-axis motion sensor...");
    uint8_t c = imu.getMPU9250ID();
    Serial.print("MPU9250 ");
    Serial.print("I AM ");
    Serial.print(c, HEX);
    Serial.print(" I should be ");
    Serial.println(0x71, HEX);
    delay(1000);

    if (c == 0x71 ) { // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
    
        Serial.println("MPU9250 is online...");

        imu.resetMPU9250(); // start by resetting MPU9250

        float SelfTest[6];    // holds results of gyro and accelerometer self test

        imu.SelfTest(SelfTest); // Start by performing self test and reporting values

        Serial.print("x-axis self test: acceleration trim within : "); 
        Serial.print(SelfTest[0],1); 
        Serial.println("% of factory value");
        Serial.print("y-axis self test: acceleration trim within : "); 
        Serial.print(SelfTest[1],1); 
        Serial.println("% of factory value");
        Serial.print("z-axis self test: acceleration trim within : "); 
        Serial.print(SelfTest[2],1); 
        Serial.println("% of factory value");
        Serial.print("x-axis self test: gyration trim within : "); 
        Serial.print(SelfTest[3],1); 
        Serial.println("% of factory value");
        Serial.print("y-axis self test: gyration trim within : "); 
        Serial.print(SelfTest[4],1); 
        Serial.println("% of factory value");
        Serial.print("z-axis self test: gyration trim within : "); 
        Serial.print(SelfTest[5],1); 
        Serial.println("% of factory value");
        delay(1000);

        // get sensor resolutions, only need to do this once
        aRes = imu.getAres(Ascale);
        gRes = imu.getGres(Gscale);
        mRes = imu.getMres(Mscale);

        // XXX should be able to call imu.calibrateMPU9250() here, but it will break master mode


        imu.initMPU9250(Ascale, Gscale, sampleRate); 
        Serial.println("MPU9250 initialized for active data mode...."); 

        // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
        uint8_t d = imu.getAK8963CID();//whoAmIAK8963();

        Serial.print("AK8963 ");
        Serial.print("I AM ");
        Serial.print(d, HEX);
        Serial.print(" I should be ");
        Serial.println(0x48, HEX);
        delay(1000); 

        // Get magnetometer calibration from AK8963 ROM
        imu.initAK8963(Mscale, Mmode, magCalibration);
        Serial.println("AK8963 initialized for active data mode...."); 

        // Comment out if using pre-measured, pre-stored offset biases
        /*
           Serial.println("Mag Calibration: Wave device in a figure eight until done!");
           delay(4000);
           imu.magcalMPU9250(magBias, magScale);
           Serial.println("Mag Calibration done!");
           Serial.println("AK8963 mag biases (mG)");
           Serial.println(magBias[0]);
           Serial.println(magBias[1]);
           Serial.println(magBias[2]); 
           Serial.println("AK8963 mag scale (mG)");
           Serial.println(magScale[0]);
           Serial.println(magScale[1]);
           Serial.println(magScale[2]); 
           delay(2000); // add delay to see results before serial spew of data
         */
        Serial.println("Calibration values: ");
        Serial.print("X-Axis sensitivity adjustment value ");
        Serial.println(magCalibration[0], 2);
        Serial.print("Y-Axis sensitivity adjustment value ");
        Serial.println(magCalibration[1], 2);
        Serial.print("Z-Axis sensitivity adjustment value ");
        Serial.println(magCalibration[2], 2);

        attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for intPin output of MPU9250
    }

    else {

        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);
        while(1) ; // Loop forever if communication doesn't happen
    }

    digitalWrite(ledPin, LOW); // turn off led when using flash memory

    delay(3000);                // wait a bit before looping
}

void loop(void)
{
    // If intPin goes high, either all data registers have new data
    // or the accel wake on motion threshold has been crossed
    if(gotNewData) {   // On interrupt, read data

        Serial.println(millis());

        gotNewData = false;     // reset gotNewData flag

    }
 }
