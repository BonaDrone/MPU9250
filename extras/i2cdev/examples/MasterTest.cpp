/* 07/6/2017 Copyright Tlera Corporation
 *  
 * Created by Kris Winer
 *
 * Adapted for WiringPi by Simon D. Levy April 2018
 *  
 * Demonstrate basic MPU-9250 functionality in master mode including
 * parameterizing the register addresses, initializing the sensor, getting
 * properly scaled accelerometer, gyroscope, and magnetometer data out. 
 *
 * SDA and SCL have 4K7 pull-up resistors (to 3.3V).
 *
 * Library may be used freely and without limit with attribution.
 */

#include <MPU9250.h>
#include <stdio.h>
#include <unistd.h>

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

static const Gscale_t GSCALE    = GFS_250DPS;
static const Ascale_t ASCALE    = AFS_2G;
static const Mscale_t MSCALE    = MFS_16BITS;
static const Mmode_t  MMODE     = M_100Hz;
static const uint8_t SAMPLE_RATE_DIVISOR = 0x04;         

// scale resolutions per LSB for the sensors
static float aRes, gRes, mRes;

// Factory mag calibration and mag bias
static float   magCalibration[3]; 

// Bias corrections for gyro and accelerometer. These can be measured once and
// entered here or can be calculated each time the device is powered on.
static float gyroBias[3], accelBias[3], magBias[3]={0,0,0}, magScale[3]={1,1,1};      

// Instantiate MPU9250 class in master mode
static MPU9250_Master imu;

void setup()
{
    // Open a connection to the MPU9250
    imu.begin();

    usleep(100000);

    // Configure the MPU9250 
    // Read the WHO_AM_I register, this is a good test of communication
    printf("MPU9250 9-axis motion sensor...\n");
    uint8_t c = imu.getMPU9250ID();
    printf("MPU9250  I AM %02X  I should be 0x71\n", c);
    usleep(1000000);

    if (c == 0x71 ) { // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
    
        printf("MPU9250 is online...\n");

        imu.resetMPU9250(); // start by resetting MPU9250

        float SelfTest[6];    // holds results of gyro and accelerometer self test

        imu.SelfTest(SelfTest); // Start by performing self test and reporting values

        printf("x-axis self test: acceleration trim within : %+3.3f%% of factory value\n", SelfTest[0]); 
        printf("y-axis self test: acceleration trim within : %+3.3f%% of factory value\n", SelfTest[1]); 
        printf("z-axis self test: acceleration trim within : %+3.3f%% of factory value\n", SelfTest[2]); 
        printf("x-axis self test: gyration trim within : %+3.3f%% of factory value\n", SelfTest[3]); 
        printf("y-axis self test: gyration trim within : %+3.3f%% of factory value\n", SelfTest[4]); 
        printf("z-axis self test: gyration trim within : %+3.3f%% of factory value\n", SelfTest[5]); 
        usleep(1000000);

        // get sensor resolutions, only need to do this once
        aRes = imu.getAres(ASCALE);
        gRes = imu.getGres(GSCALE);
        mRes = imu.getMres(MSCALE);

        // Comment out if using pre-measured, pre-stored offset accel/gyro biases
        imu.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
        printf("accel biases (mg)\n");
        printf("%f\n", 1000.*accelBias[0]);
        printf("%f\n", 1000.*accelBias[1]);
        printf("%f\n", 1000.*accelBias[2]);
        printf("gyro biases (dps)\n");
        printf("%f\n", gyroBias[0]);
        printf("%f\n", gyroBias[1]);
        printf("%f\n", gyroBias[2]);
        usleep(1000000); 

        imu.initMPU9250(ASCALE, GSCALE, SAMPLE_RATE_DIVISOR); 
        printf("MPU9250 initialized for active data mode....\n"); 

        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        uint8_t d = imu.getAK8963CID();  // Read WHO_AM_I register for AK8963
        printf("AK8963  I AM 0x%02x  I should be 0x48\n", d);
        usleep(1000000); 

        // Get magnetometer calibration from AK8963 ROM
        imu.initAK8963(MSCALE, MMODE, magCalibration);
        printf("AK8963 initialized for active data mode....\n"); 

        // Comment out if using pre-measured, pre-stored offset magnetometer biases
        printf("Mag Calibration: Wave device in a figure eight until done!\n");
        usleep(4000000);
        imu.magcalMPU9250(magBias, magScale);
        printf("Mag Calibration done!\n");
        printf("AK8963 mag biases (mG)\n");
        printf("%f\n", magBias[0]);
        printf("%f\n", magBias[1]);
        printf("%f\n", magBias[2]); 
        printf("AK8963 mag scale (mG)\n");
        printf("%f\n", magScale[0]);
        printf("%f\n", magScale[1]);
        printf("%f\n", magScale[2]); 
        usleep(2000000); // add delay to see results before serial spew of data
        printf("Calibration values:\n");
        printf("X-Axis sensitivity adjustment value %+2.2f\n", magCalibration[0]);
        printf("Y-Axis sensitivity adjustment value %+2.2f\n", magCalibration[1]);
        printf("Z-Axis sensitivity adjustment value %+2.2f\n", magCalibration[2]);
    }
    else {

        printf("Could not connect to MPU9250: 0x%02x", c);
        while(true) ; // Loop forever if communication doesn't happen
    }

    usleep(3000000);                // wait a bit before looping
}

void loop()
{  
    static int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
    static float ax, ay, az, gx, gy, gz, mx, my, mz;

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
    }

    // Report at 1Hz
    //static uint32_t msec_prev;
    //uint32_t msec_curr = millis();

    //if (msec_curr-msec_prev > 1000) {

        //msec_prev = msec_curr;

        printf("ax = %d  ay = %d  az = %d mg\n", (int)(1000*ax), (int)(1000*ay), (int)(1000*az));
        printf("gx = %+2.2f  gy = %+2.2f  gz = %+2.2f deg/s\n", gx, gy, gz);
        printf("mx = %d  my = %d  mz = %d mG\n", (int)mx, (int)my, (int)mz);

        float temperature = ((float) MPU9250Data[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade

        // Print temperature in degrees Centigrade      
        printf("Gyro temperature is %+1.1f degrees C\n", temperature);  
   // }

     usleep(1000000);

}
