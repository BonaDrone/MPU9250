/*  Implementation of MPU6050 class library
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU6050.h"
#include "CrossPlatformI2C_Core.h"

// One ifdef needed to support delay() cross-platform
#if defined(ARDUINO)
#include <Arduino.h>

#elif defined(__arm__) 
#if defined(STM32F303)
extern "C" { void delay(uint32_t msec); }
#else
#include <wiringPi.h>
#endif

#else
void delay(uint32_t msec);
#endif

#include <math.h>

#include <stdio.h>

void MPU6050::writeMPURegister(uint8_t subAddress, uint8_t data)
{
    cpi2c_writeRegister(_i2c, subAddress, data);
}

void MPU6050::readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    cpi2c_readRegisters(_i2c, subAddress, count, dest);
}


MPU6050::MPU6050(Ascale_t ascale, Gscale_t gscale)
{
    _aRes = getAres(ascale);
    _gRes = getGres(gscale);

    _aScale = ascale;
    _gScale = gscale;
}

MPU_Error_t MPU6050::begin(uint8_t bus)
{
    _i2c = cpi2c_open(MPU6050_ADDRESS, bus);

    if (getId() != 0x68) {
        return MPU_ERROR_IMU_ID;
    }

    float tolerances[6]; 
    selfTest(tolerances); 
    for (uint8_t k=0; k<6; ++k) {
        if (tolerances[k] >= 1.0f) {
            return MPU_ERROR_SELFTEST;
        }
    }

    calibrate(_accelBias, _gyroBias);

    init(_aScale, _gScale);

    return MPU_ERROR_NONE;
}

bool MPU6050::checkNewData(void)
{
    return (bool)(readMPURegister(INT_STATUS) & 0x01);
}

void MPU6050::readAccelerometer(float & ax, float & ay, float & az)
{
    // x/y/z accel register data stored here    
    uint8_t rawData[6];  

    // Read the six raw data registers into data array
    readMPURegisters(ACCEL_XOUT_H, 6, &rawData[0]);  

    // Turn the MSB and LSB into a signed 16-bit value
    int16_t x = (int16_t)((rawData[0] << 8) | rawData[1]) ;  
    int16_t y = (int16_t)((rawData[2] << 8) | rawData[3]) ;
    int16_t z = (int16_t)((rawData[4] << 8) | rawData[5]) ;

    // Convert to Gs
    ax = (float)x*_aRes - _accelBias[0];  
    ay = (float)y*_aRes - _accelBias[1];   
    az = (float)z*_aRes - _accelBias[2];  
}

void MPU6050::readGyrometer(float & gx, float & gy, float & gz)
{
    // x/y/z gyro register data stored here
    uint8_t rawData[6];  

    // Read the six raw data registers sequentially into data array
    readMPURegisters(GYRO_XOUT_H, 6, &rawData[0]);  

    // Turn the MSB and LSB into a signed 16-bit value
    int16_t x = (int16_t)((rawData[0] << 8) | rawData[1]) ;  
    int16_t y = (int16_t)((rawData[2] << 8) | rawData[3]) ;
    int16_t z = (int16_t)((rawData[4] << 8) | rawData[5]) ;

    // Calculate the gyro value into actual degrees per second
    gx = (float)x*_gRes - _gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)y*_gRes - _gyroBias[1];  
    gz = (float)z*_gRes - _gyroBias[2];   
}

float MPU6050::readTemperature()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    readMPURegisters(TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    int16_t t = ((int16_t)rawData[0]) << 8 | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
    return ((float) t) / 340. + 36.53; // Temperature in degrees Centigrade
}



// Configure the motion detection control for low power accelerometer mode
void MPU6050::lowPowerAccelOnly()
{
    // The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
    // Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
    // above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
    // threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
    // consideration for these threshold evaluations; otherwise, the flags would be set all the time!

    uint8_t c = readMPURegister(PWR_MGMT_1);
    writeMPURegister(PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
    writeMPURegister(PWR_MGMT_1, c |  0x30); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

    c = readMPURegister(PWR_MGMT_2);
    writeMPURegister(PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
    writeMPURegister(PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running

    c = readMPURegister(ACCEL_CONFIG);
    writeMPURegister(ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
    // Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
    writeMPURegister(ACCEL_CONFIG,  c | 0x00);  // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter

    c = readMPURegister(CONFIG);
    writeMPURegister(CONFIG, c & ~0x07); // Clear low-pass filter bits [2:0]
    writeMPURegister(CONFIG, c |  0x00);  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate

    c = readMPURegister(INT_ENABLE);
    writeMPURegister(INT_ENABLE, c & ~0xFF);  // Clear all interrupts
    writeMPURegister(INT_ENABLE, 0x40);  // Enable motion threshold (bits 5) interrupt only

    // Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
    // for at least the counter duration
    writeMPURegister(MOT_THR, 0x80); // Set motion detection to 0.256 g; LSB = 2 mg
    writeMPURegister(MOT_DUR, 0x01); // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate

    delay (100);  // Add delay for accumulation of samples

    c = readMPURegister(ACCEL_CONFIG);
    writeMPURegister(ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
    writeMPURegister(ACCEL_CONFIG, c |  0x07);  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance

    c = readMPURegister(PWR_MGMT_2);
    writeMPURegister(PWR_MGMT_2, c & ~0xC7); // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
    writeMPURegister(PWR_MGMT_2, c |  0x47); // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])

    c = readMPURegister(PWR_MGMT_1);
    writeMPURegister(PWR_MGMT_1, c & ~0x20); // Clear sleep and cycle bit 5
    writeMPURegister(PWR_MGMT_1, c |  0x20); // Set cycle bit 5 to begin low power accelerometer motion interrupts

}

void MPU6050::init(Ascale_t ascale, Gscale_t gscale)
{
    // wake up device-don't need this here if using calibration function below
    //  writeMPURegister(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    //  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

    // get stable time source
    writeMPURegister(PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
    writeMPURegister(CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeMPURegister(SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c =  readMPURegister(GYRO_CONFIG);
    writeMPURegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
    writeMPURegister(GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeMPURegister(GYRO_CONFIG, c | gscale << 3); // Set full scale range for the gyro

    // Set accelerometer configuration
    c =  readMPURegister(ACCEL_CONFIG);
    writeMPURegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
    writeMPURegister(ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeMPURegister(ACCEL_CONFIG, c | ascale << 3); // Set full scale range for the accelerometer

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeMPURegister(INT_PIN_CFG, 0x22);
    writeMPURegister(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU6050::calibrate(float accelBias[3], float gyroBias[3])
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    writeMPURegister(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeMPURegister(PWR_MGMT_1, 0x01);
    writeMPURegister(PWR_MGMT_2, 0x00);
    delay(200);

    // Configure device for bias calculation
    writeMPURegister(INT_ENABLE, 0x00);   // Disable all interrupts
    writeMPURegister(FIFO_EN, 0x00);      // Disable FIFO
    writeMPURegister(PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeMPURegister(I2C_MST_CTRL, 0x00); // Disable I2C master
    writeMPURegister(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeMPURegister(USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeMPURegister(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    writeMPURegister(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeMPURegister(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeMPURegister(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeMPURegister(USER_CTRL, 0x40);   // Enable FIFO
    writeMPURegister(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeMPURegister(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    readMPURegisters(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readMPURegisters(FIFO_R_W, 12, &data[0]); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];

    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if (accel_bias[2] > 0L) {
        accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
    }
    else {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4)       & 0xFF;
    data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4)       & 0xFF;

    // Push gyro biases to hardware registers
    writeMPURegister(XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
    writeMPURegister(XG_OFFS_USRL, data[1]);
    writeMPURegister(YG_OFFS_USRH, data[2]);
    writeMPURegister(YG_OFFS_USRL, data[3]);
    writeMPURegister(ZG_OFFS_USRH, data[4]);
    writeMPURegister(ZG_OFFS_USRL, data[5]);

    gyroBias[0] = (float) gyro_bias[0] / (float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    gyroBias[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
    gyroBias[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    readMPURegisters(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readMPURegisters(YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readMPURegisters(ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++) {
        if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    writeMPURegister(XA_OFFSET_H, data[0]); // might not be supported in MPU6050
    writeMPURegister(XA_OFFSET_L_TC, data[1]);
    writeMPURegister(YA_OFFSET_H, data[2]);
    writeMPURegister(YA_OFFSET_L_TC, data[3]);
    writeMPURegister(ZA_OFFSET_H, data[4]);
    writeMPURegister(ZA_OFFSET_L_TC, data[5]);

    // Output scaled accelerometer biases for manual subtraction in the main program
    accelBias[0] = (float)accel_bias[0] / (float)accelsensitivity;
    accelBias[1] = (float)accel_bias[1] / (float)accelsensitivity;
    accelBias[2] = (float)accel_bias[2] / (float)accelsensitivity;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050::selfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[4];
    uint8_t selfTest[6];
    float factoryTrim[6];

    // Configure the accelerometer for self-test
    writeMPURegister(ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    writeMPURegister(GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    delay(250);  // Delay a while to let the device execute the self-test
    rawData[0] = readMPURegister(SELF_TEST_X); // X-axis self-test results
    rawData[1] = readMPURegister(SELF_TEST_Y); // Y-axis self-test results
    rawData[2] = readMPURegister(SELF_TEST_Z); // Z-axis self-test results
    rawData[3] = readMPURegister(SELF_TEST_A); // Mixed-axis self-test results
    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ; // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results first
    selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[0] - 1.0) / 30.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[1] - 1.0) / 30.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[2] - 1.0) / 30.0))); // FT[Za] factory trim calculation
    factoryTrim[3] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[3] - 1.0) ));         // FT[Xg] factory trim calculation
    factoryTrim[4] =  (-25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[4] - 1.0) ));         // FT[Yg] factory trim calculation
    factoryTrim[5] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[5] - 1.0) ));         // FT[Zg] factory trim calculation

    //  Output self-test results and factory trim calculation if desired
    //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
    //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
    //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
    //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        destination[i] = 100.0 + 100.0 * ((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i]; // Report percent differences
    }
}


