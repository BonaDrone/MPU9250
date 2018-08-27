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

        void        readAccelerometer(float & ax, float & ay, float & az);
        void        readGyrometer(float & gx, float & gy, float & gz);
        float       readTemperature(void);

    protected:

        virtual void writeMPURegister(uint8_t subAddress, uint8_t data) override;

        virtual void readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;

        virtual uint8_t xAOffsetH(void) override;
        virtual uint8_t yAOffsetH(void) override;
        virtual uint8_t zAOffsetH(void) override;

    private:

        const uint8_t MPU6050_ADDRESS = 0x68;

        const uint8_t XGOFFS_TC        		= 0x00; // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD;
        const uint8_t YGOFFS_TC        		= 0x01;
        const uint8_t ZGOFFS_TC        		= 0x02;
        const uint8_t X_FINE_GAIN      		= 0x03; // [7:0] fine gain
        const uint8_t Y_FINE_GAIN      		= 0x04;
        const uint8_t Z_FINE_GAIN      		= 0x05;

        const uint8_t XA_OFFSET_H      		= 0x06; // User-defined trim values for accelerometer
        const uint8_t XA_OFFSET_L_TC   		= 0x07;
        const uint8_t YA_OFFSET_H      		= 0x08;
        const uint8_t YA_OFFSET_L_TC   		= 0x09;
        const uint8_t ZA_OFFSET_H      		= 0x0A;
        const uint8_t ZA_OFFSET_L_TC   		= 0x0B;

        const uint8_t SELF_TEST_X      		= 0x0D;
        const uint8_t SELF_TEST_Y      		= 0x0E;
        const uint8_t SELF_TEST_Z      		= 0x0F;
        const uint8_t SELF_TEST_A      		= 0x10;

        const uint8_t FF_THR           		= 0x1D;  // Free-fall
        const uint8_t FF_DUR           		= 0x1E;  // Free-fall
        const uint8_t MOT_THR          		= 0x1F;  // Motion detection threshold bits [7:0]
        const uint8_t MOT_DUR          		= 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
        const uint8_t ZMOT_THR         		= 0x21;  // Zero-motion detection threshold bits [7:0]
        const uint8_t ZRMOT_DUR        		= 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

        void     selfTest(float * destination);

        void     init(Ascale_t ascale, Gscale_t gscale);

        // Cross-platform support
        uint8_t  _i2c;
}; 
