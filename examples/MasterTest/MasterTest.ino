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

// Factory mag calibration and mag bias
static float   magCalibration[3]; 

// Bias corrections for gyro and accelerometer. These can be measured once and
// entered here or can be calculated each time the device is powered on.
static float gyroBias[3], accelBias[3], magBias[3]={0,0,0}, magScale[3]={1,1,1};      

// Create a byte-transfer object for Arduino I^2C
ArduinoI2C bt;

// Instantiate MPU9250 class in pass-thru mode XXX should be master
static MPU9250Passthru imu = MPU9250Passthru(&bt); 

// Device address when ADO = 0
static const uint8_t MPU9250_ADDRESS  = 0x68;  

// MPU9250 registers
const uint8_t EXT_SENS_DATA_00 = 0x49;

const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;

// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_WHO_AM_I = 0x00;

// reads registers from the AK8963  in master mode
static void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    // set slave 0 to the AK8963 and set for read
    bt.writeRegister(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_I2C_ADDR|I2C_READ_FLAG);

    // set the register to the desired AK8963 sub address
    bt.writeRegister(MPU9250_ADDRESS, I2C_SLV0_REG, subAddress);

    // enable I2C and request the bytes
    bt.writeRegister(MPU9250_ADDRESS, I2C_SLV0_CTRL, I2C_SLV0_EN|count);

    delay(1); // takes some time for these registers to fill

    // read the bytes off the MPU9250 EXT_SENS_DATA registers
    bt.readRegisters(MPU9250_ADDRESS, EXT_SENS_DATA_00, count, dest); 
}

static int whoAmIAK8963()
{
    uint8_t buffer = 0;

    // read the WHO AM I register
    readAK8963Registers(AK8963_WHO_AM_I, 1, &buffer);

    // return the register value
    return buffer;
}

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

    if (c == 0x71 ) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
    {  
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


        //imu.initMPU9250(Ascale, Gscale, sampleRate); 
        //Serial.println("MPU9250 initialized for active data mode...."); 

        // enable I2C master mode
        bt.writeRegister(MPU9250_ADDRESS, USER_CTRL, I2C_MST_EN);

        // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
        uint8_t d = whoAmIAK8963();

        Serial.print("AK8963 ");
        Serial.print("I AM ");
        Serial.print(d, HEX);
        Serial.print(" I should be ");
        Serial.println(0x48, HEX);
        delay(1000); 
    }
}

void loop(void)
{
}
