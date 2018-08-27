/*  Header file for MPU9250 class library
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. Levy 19 April 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#pragma once

#include <stdint.h>

#include "MPU.h"

typedef enum {

    MFS_14BITS, // 0.6 mG per LSB
    MFS_16BITS  // 0.15 mG per LSB

} Mscale_t;

typedef enum {

    M_8Hz   = 0x02,
    M_100Hz = 0x06

} Mmode_t;

class MPU9250 : public MPUIMU {

    public: 

        void    accelWakeOnMotion(void);
        bool    checkWakeOnMotion(void);
        void    calibrateMagnetometer(void);
        void    gyroMagSleep();
        void    gyroMagWake(Mmode_t mmode);
        void    readAccelerometer(float & ax, float & ay, float & az);
        void    readGyrometer(float & ax, float & ay, float & az);
        void    readMagnetometer(float & mx, float & my, float & mz);
        float   readTemperature(void);

    protected:

        MPU9250(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor, bool passthru);

        MPU_Error_t runTests(void);

        //static const uint8_t MPU9250_ADDRESS = 0x69; // When AD0 = 1
        static const uint8_t MPU9250_ADDRESS = 0x68;   // When AD0 = 0
        static const uint8_t AK8963_ADDRESS  = 0x0C;

        uint8_t _mpu;

        MPU_Error_t begin(uint8_t bus=1);

        void initMPU9250(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor, bool passthru);

        virtual void writeAK8963Register(uint8_t subAddress, uint8_t data) = 0;

        virtual void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t * dest) = 0;

        virtual void readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

        virtual uint8_t readRegister(uint8_t address, uint8_t subAddress);

        virtual void writeRegister(uint8_t address, uint8_t subAddress, uint8_t data);

        // See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 
        // for registers not listed in above document; the MPU9250 and MPU9150 are virtually identical but 
        // the latter has a different register map
        
        //Magnetometer Registers
        const uint8_t WHO_AM_I_AK8963   = 0x00; // should return  = 0x48
        const uint8_t INFO              = 0x01;
        const uint8_t AK8963_ST1        = 0x02; // data ready status bit 0
        const uint8_t AK8963_XOUT_L     = 0x03;  // data
        const uint8_t AK8963_XOUT_H     = 0x04;
        const uint8_t AK8963_YOUT_L     = 0x05;
        const uint8_t AK8963_YOUT_H     = 0x06;
        const uint8_t AK8963_ZOUT_L     = 0x07;
        const uint8_t AK8963_ZOUT_H     = 0x08;
        const uint8_t AK8963_ST2        = 0x09;  // Data overflow bit 3 and data read error status bit 2
        const uint8_t AK8963_CNTL       = 0x0A;  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
        const uint8_t AK8963_ASTC       = 0x0C;  // Self test control
        const uint8_t AK8963_I2CDIS     = 0x0F;  // I2C disable
        const uint8_t AK8963_ASAX       = 0x10;  // Fuse ROM x-axis sensitivity adjustment value
        const uint8_t AK8963_ASAY       = 0x11;  // Fuse ROM y-axis sensitivity adjustment value
        const uint8_t AK8963_ASAZ       = 0x12;  // Fuse ROM z-axis sensitivity adjustment value

        const uint8_t SELF_TEST_X_GYRO  = 0x00;                  
        const uint8_t SELF_TEST_Y_GYRO  = 0x01;                                                                          
        const uint8_t SELF_TEST_Z_GYRO  = 0x02;

        /*
           const uint8_t X_FINE_GAIN       = 0x03; // [7:0] fine gain
           const uint8_t Y_FINE_GAIN       = 0x04;
           const uint8_t Z_FINE_GAIN       = 0x05;
           const uint8_t XA_OFFSET_H       = 0x06; // User-defined trim values for accelerometer
           const uint8_t XA_OFFSET_L_TC    = 0x07;
           const uint8_t YA_OFFSET_H       = 0x08;
           const uint8_t YA_OFFSET_L_TC    = 0x09;
           const uint8_t ZA_OFFSET_H       = 0x0A;
           const uint8_t ZA_OFFSET_L_TC    = 0x0B; 
         */

        const uint8_t SELF_TEST_X_ACCEL  = 0x0D;
        const uint8_t SELF_TEST_Y_ACCEL  = 0x0E;    
        const uint8_t SELF_TEST_Z_ACCEL  = 0x0F;

        const uint8_t SELF_TEST_A       = 0x10;

        const uint8_t XG_OFFSET_H       = 0x13;  // User-defined trim values for gyroscope
        const uint8_t XG_OFFSET_L       = 0x14;
        const uint8_t YG_OFFSET_H       = 0x15;
        const uint8_t YG_OFFSET_L       = 0x16;
        const uint8_t ZG_OFFSET_H       = 0x17;
        const uint8_t ZG_OFFSET_L       = 0x18;
        const uint8_t SMPLRT_DIV        = 0x19;
        const uint8_t CONFIG            = 0x1A;
        const uint8_t GYRO_CONFIG       = 0x1B;
        const uint8_t ACCEL_CONFIG      = 0x1C;
        const uint8_t ACCEL_CONFIG2     = 0x1D;
        const uint8_t LP_ACCEL_ODR      = 0x1E;
        const uint8_t WOM_THR           = 0x1F;   

        const uint8_t MOT_DUR           = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
        const uint8_t ZMOT_THR          = 0x21;  // Zero-motion detection threshold bits [7:0]
        const uint8_t ZRMOT_DUR         = 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

        const uint8_t XA_OFFSET_H       = 0x77;
        const uint8_t XA_OFFSET_L       = 0x78;
        const uint8_t YA_OFFSET_H       = 0x7A;
        const uint8_t YA_OFFSET_L       = 0x7B;
        const uint8_t ZA_OFFSET_H       = 0x7D;
        const uint8_t ZA_OFFSET_L       = 0x7E;
        const uint8_t I2C_SLV0_EN       = 0x80;

        const uint8_t I2C_READ_FLAG     = 0x80;
        const uint8_t I2C_MST_EN        = 0x20;

        void    writeMPURegister(uint8_t subAddress, uint8_t data);

        void    readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * data);

        uint8_t readMPURegister(uint8_t subAddress);

        uint8_t readAK8963Register(uint8_t subAddress);
        
        Mscale_t _mScale;
        Mmode_t  _mMode;
        uint8_t  _sampleRateDivisor;

        bool    _passthru;
        float   _mRes;
        float   _fuseROMx;
        float   _fuseROMy;
        float   _fuseROMz;
        float   _magCalibration[3];

    private:

        void    calibrate(float accelBias[6], float gyroBias[6]);
        float   getAres(Ascale_t ascale);
        float   getGres(Gscale_t gscale);
        uint8_t getId(void);
        void    selfTest(float tolerances[6]);

        uint8_t getAK8963CID(void);
        float   getMres(Mscale_t mscale);
        void    reset(void);
        void    readMagData(int16_t * destination);
        void    initAK8963(Mscale_t mscale, Mmode_t Mmode, float * magCalibration);


        // These can be overridden by calibrateMagnetometer()
        float _magBias[3] = {0,0,0};
        float _magScale[3] = {1,1,1};

}; // class MPU9250

class MPU9250_Passthru : public MPU9250 {

    public:

        MPU9250_Passthru(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor);

        MPU_Error_t begin(uint8_t i2cbus=1);

        bool checkNewAccelGyroData(void);

        bool checkNewMagData(void);

    private:

        virtual void writeAK8963Register(uint8_t subAddress, uint8_t data) override;

        virtual void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest) override;

    protected:

        uint8_t _mag;

}; // class MPU9250_Passthru

class MPU9250_Master : public MPU9250 {

    public:

        MPU9250_Master(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor);

        virtual MPU_Error_t begin(uint8_t i2cbus=1);

        bool checkNewData(void);

    protected:

        virtual void writeAK8963Register(uint8_t subAddress, uint8_t data) override;

        virtual void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest) override;

    private:

        void initMPU9250(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor);
};

class MPU9250_SPI : public MPU9250 {

    public:

        MPU9250_SPI(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor);

        virtual MPU_Error_t begin(void);

        void initMPU9250(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor);

        bool checkNewData(void);

    protected:

        virtual void writeAK8963Register(uint8_t subAddress, uint8_t data) override;

        virtual void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest) override;
};
