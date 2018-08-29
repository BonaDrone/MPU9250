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

#include "MPU6500.h"

typedef enum {

    MFS_14BITS, // 0.6 mG per LSB
    MFS_16BITS  // 0.15 mG per LSB

} Mscale_t;

typedef enum {

    M_8Hz   = 0x02,
    M_100Hz = 0x06

} Mmode_t;

// MPU9250 is MPU6500 plus magnetometer
class MPU9250 : public MPU6500 {

    public: 

        void  accelWakeOnMotion(void);

        bool  checkWakeOnMotion(void);

        void  calibrateMagnetometer(void);

        void  gyroMagSleep();

        void  gyroMagWake(Mmode_t mmode);

        void  readGyrometer(float & ax, float & ay, float & az);

        void  readMagnetometer(float & mx, float & my, float & mz);

        float readTemperature(void);

    protected:

        MPU9250(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor, bool passthru);

        MPU_Error_t runTests(void);

        static const uint8_t AK8963_ADDRESS  = 0x0C;

        uint8_t _mpu;

        MPU_Error_t begin(uint8_t bus=1);

        void initMPU6500(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor, bool passthru);

        virtual void pushGyroBiases(uint8_t data[12]) override;

        virtual void readAccelOffsets(uint8_t data[12], int32_t accel_bias_reg[3]) override;

        virtual void writeAK8963Register(uint8_t subAddress, uint8_t data) = 0;

        virtual void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t * dest) = 0;

        virtual void readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

        virtual uint8_t readRegister(uint8_t address, uint8_t subAddress);

        virtual void writeRegister(uint8_t address, uint8_t subAddress, uint8_t data);

        // See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 
        // for registers not listed in above document; the MPU9250 and MPU9150 are virtually identical but 
        // the latter has a different register map
        
        // Magnetometer Registers
        const uint8_t WHO_AM_I_AK8963   = 0x00; 
        const uint8_t INFO              = 0x01;
        const uint8_t AK8963_ST1        = 0x02; 
        const uint8_t AK8963_XOUT_L     = 0x03; 
        const uint8_t AK8963_XOUT_H     = 0x04;
        const uint8_t AK8963_YOUT_L     = 0x05;
        const uint8_t AK8963_YOUT_H     = 0x06;
        const uint8_t AK8963_ZOUT_L     = 0x07;
        const uint8_t AK8963_ZOUT_H     = 0x08;
        const uint8_t AK8963_ST2        = 0x09; 
        const uint8_t AK8963_CNTL       = 0x0A;  
        const uint8_t AK8963_ASTC       = 0x0C; 
        const uint8_t AK8963_I2CDIS     = 0x0F;
        const uint8_t AK8963_ASAX       = 0x10;
        const uint8_t AK8963_ASAY       = 0x11;
        const uint8_t AK8963_ASAZ       = 0x12;

        const uint8_t SELF_TEST_X_ACCEL = 0x0D;
        const uint8_t SELF_TEST_Y_ACCEL = 0x0E;    
        const uint8_t SELF_TEST_Z_ACCEL = 0x0F;

        const uint8_t SELF_TEST_A       = 0x10;

        const uint8_t GYRO_CONFIG       = 0x1B;
        const uint8_t ACCEL_CONFIG2     = 0x1D;
        const uint8_t LP_ACCEL_ODR      = 0x1E;
        const uint8_t WOM_THR           = 0x1F;   

        const uint8_t MOT_DUR           = 0x20;  
        const uint8_t ZMOT_THR          = 0x21;  
        const uint8_t ZRMOT_DUR         = 0x22;  

        const uint8_t XG_OFFSET_H       = 0x13; 
        const uint8_t XG_OFFSET_L       = 0x14;
        const uint8_t YG_OFFSET_H       = 0x15;
        const uint8_t YG_OFFSET_L       = 0x16;
        const uint8_t ZG_OFFSET_H       = 0x17;
        const uint8_t ZG_OFFSET_L       = 0x18;
        const uint8_t XA_OFFSET_H       = 0x77;
        const uint8_t XA_OFFSET_L       = 0x78;
        const uint8_t YA_OFFSET_H       = 0x7A;
        const uint8_t YA_OFFSET_L       = 0x7B;
        const uint8_t ZA_OFFSET_H       = 0x7D;
        const uint8_t ZA_OFFSET_L       = 0x7E;

        const uint8_t I2C_SLV0_EN       = 0x80;

        const uint8_t I2C_READ_FLAG     = 0x80;
        const uint8_t I2C_MST_EN        = 0x20;

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

        virtual void writeMPURegister(uint8_t subAddress, uint8_t data) override;

        virtual void readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;

    private:

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
