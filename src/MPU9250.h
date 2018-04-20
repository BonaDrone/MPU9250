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

enum {
  AFS_2G,
  AFS_4G,  
  AFS_8G,  
  AFS_16G 
};

enum {
  GFS_250DPS,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum {
  MFS_14BITS, // 0.6 mG per LSB
  MFS_16BITS  // 0.15 mG per LSB
};

const uint8_t M_8Hz   = 0x02;
const uint8_t M_100Hz = 0x06;

class MPU9250
{
    public: 

        uint8_t getMPU9250ID(void);
        uint8_t getAK8963CID(void);
        void    resetMPU9250(void);
        void    initMPU9250(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate);
        void    initAK8963(uint8_t Mscale, uint8_t Mmode, float * destination);
        float   getAres(uint8_t Ascale);
        float   getGres(uint8_t Gscale);
        float   getMres(uint8_t Mscale);
        void    magcalMPU9250(float * dest1, float * dest2);
        void    calibrateMPU9250(float * dest1, float * dest2);
        void    readMPU9250Data(int16_t * destination);
        void    readAccelData(int16_t * destination);
        void    readGyroData(int16_t * destination);
        bool    checkNewAccelGyroData(void);
        bool    checkNewMagData(void);
        void    readMagData(int16_t * destination);
        int16_t readGyroTempData(void);
        void    gyromagSleep(void);
        void    gyromagWake(uint8_t Mmode);
        void    accelWakeOnMotion(void);
        bool    checkWakeOnMotion(void);
        void    I2Cscan(void);
        void    SelfTest(float * destination);

    private:

        void    writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
        uint8_t readByte(uint8_t address, uint8_t subAddress);
        void    readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

        float   _aRes;
        float   _gRes;
        float   _mRes;
        uint8_t _Mmode;
        float   _fuseROMx;
        float   _fuseROMy;
        float   _fuseROMz;
        float   _magCalibration[3];
};
