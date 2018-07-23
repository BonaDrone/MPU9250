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


#if defined(__MK20DX256__)  
#include <i2c_t3.h>   
#else
#include <Wire.h>   
#endif

#include "MPU9250.h"

/*
   MPU9250 Configuration

   Specify sensor full scale

   Choices are:

Gscale: GFS_250 = 250 dps, GFS_500 = 500 dps, GFS_1000 = 1000 dps, GFS_2000DPS = 2000 degrees per second gyro full scale
Ascale: AFS_2G = 2 g, AFS_4G = 4 g, AFS_8G = 8 g, and AFS_16G = 16 g accelerometer full scale
Mscale: MFS_14BITS = 0.6 mG per LSB and MFS_16BITS = 0.15 mG per LSB
Mmode:  Mmode = M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
SAMPLE_RATE_DIVISOR = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
 */
static const Gscale_t GSCALE     = GFS_250DPS;
static const Ascale_t ASCALE     = AFS_2G;
static const Mscale_t MSCALE     = MFS_16BITS;
static const Mmode_t  MMODE      = M_100Hz;
static const uint8_t SAMPLE_RATE_DIVISOR = 0x04;         

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

// Bias corrections for gyro and accelerometer. These can be measured once and
// entered here or can be calculated each time the device is powered on.
static float gyroBias[3], accelBias[3], magBias[3]={0,0,0}, magScale[3]={1,1,1};      

// Create a byte-transfer object for Arduino I^2C
ArduinoI2C mpu(MPU9250::MPU9250_ADDRESS);

// Factory mag calibration and mag bias
static float magCalibration[3]; 

// Instantiate MPU9250 class in master mode
static MPU9250Master imu = MPU9250Master(&mpu); 

void setup(void)
{
    // Start serial comms
    Serial.begin(115200);
    delay(1000);

    // Start I^2C
#if defined(__MK20DX256__)  
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_100); 
#else
    Wire.begin(); 
    Wire.setClock(400000); 
#endif

    delay(1000);

    // Set up the interrupt pin, it's set as active high, push-pull
    pinMode(intPin, INPUT);

    // Start with orange led on (active HIGH)
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH); 

    imu.begin();

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
        aRes = imu.getAres(ASCALE);
        gRes = imu.getGres(GSCALE);
        mRes = imu.getMres(MSCALE);

        // Comment out if using pre-measured, pre-stored accel/gyro offset biases
        imu.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
        Serial.println("accel biases (mg)");
        Serial.println(1000.*accelBias[0]);
        Serial.println(1000.*accelBias[1]);
        Serial.println(1000.*accelBias[2]);
        Serial.println("gyro biases (dps)");
        Serial.println(gyroBias[0]);
        Serial.println(gyroBias[1]);
        Serial.println(gyroBias[2]);
        delay(1000); 

        imu.initMPU9250(ASCALE, GSCALE, SAMPLE_RATE_DIVISOR); 
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
        imu.initAK8963(MSCALE, MMODE, magCalibration);
        Serial.println("AK8963 initialized for active data mode...."); 

        // Comment out if using pre-measured, pre-stored offset magnetometer biases
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
    static int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
    static float ax, ay, az, gx, gy, gz, mx, my, mz;

    // If intPin goes high, either all data registers have new data
    // or the accel wake on motion threshold has been crossed
    if(gotNewData) {   // On interrupt, read data

        gotNewData = false;     // reset gotNewData flag

        if (imu.checkNewData())  { // data ready interrupt is detected

            imu.readMPU9250Data(MPU9250Data); // INT cleared on any read

            // Convert the accleration value into g's
            ax = (float)MPU9250Data[0]*aRes - accelBias[0];  
            ay = (float)MPU9250Data[1]*aRes - accelBias[1];   
            az = (float)MPU9250Data[2]*aRes - accelBias[2];  

            // Convert the gyro value into degrees per second
            gx = (float)MPU9250Data[4]*gRes;  
            gy = (float)MPU9250Data[5]*gRes;  
            gz = (float)MPU9250Data[6]*gRes; 

            int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

            imu.readMagData(magCount);  // Read the x/y/z adc values

            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental corrections
            // Get actual magnetometer value, this depends on scale being set
            mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  
            my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
            mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
            mx *= magScale[0];
            my *= magScale[1];
            mz *= magScale[2]; 

            // Report at 1Hz
            static uint32_t msec_prev;
            uint32_t msec_curr = millis();

            if (msec_curr-msec_prev > 1000) {

                msec_prev = msec_curr;

                Serial.print("ax = ");
                Serial.print((int)1000*ax);  
                Serial.print(" ay = ");
                Serial.print((int)1000*ay); 
                Serial.print(" az = ");
                Serial.print((int)1000*az);
                Serial.println(" mg");
                Serial.print("gx = ");
                Serial.print( gx, 2); 
                Serial.print(" gy = ");
                Serial.print( gy, 2); 
                Serial.print(" gz = ");
                Serial.print( gz, 2);
                Serial.println(" deg/s");
                Serial.print("mx = ");
                Serial.print( (int)mx ); 
                Serial.print(" my = ");
                Serial.print( (int)my ); 
                Serial.print(" mz = ");
                Serial.print( (int)mz );
                Serial.println(" mG");

                float temperature = ((float) MPU9250Data[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade

                // Print temperature in degrees Centigrade      
                Serial.print("Gyro temperature is ");  
                Serial.print(temperature, 1);  
                Serial.println(" degrees C"); 
            }
        }
    }
}
