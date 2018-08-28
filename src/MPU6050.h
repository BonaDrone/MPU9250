/*  Header file for MPU6050 class library
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. Levy 19 April 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU.h"

class MPU6050 : public MPUIMU {

    public:

        MPU6050(Ascale_t ascale, Gscale_t gscale);

        MPU_Error_t begin(uint8_t bus=1);

        bool        checkNewData(void);

        void        lowPowerAccelOnly(void);

        void        readGyrometer(float & gx, float & gy, float & gz);

        float       readTemperature(void);

    protected:

        virtual void writeMPURegister(uint8_t subAddress, uint8_t data) override;

        virtual void readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;

    private:

        const uint8_t MPU6050_ADDRESS = 0x68;

        const uint8_t SELF_TEST_X      		= 0x0D;
        const uint8_t SELF_TEST_Y      		= 0x0E;
        const uint8_t SELF_TEST_Z      		= 0x0F;
        const uint8_t SELF_TEST_A      		= 0x10;

        const uint8_t MOT_THR          		= 0x1F;  // Motion detection threshold bits [7:0]
        const uint8_t MOT_DUR          		= 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms

        void     selfTest(float * destination);

        void     init(Ascale_t ascale, Gscale_t gscale);

        // Cross-platform support
        uint8_t  _i2c;
}; 
