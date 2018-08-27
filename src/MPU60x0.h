/* MPU/6000/6050 library common header code
by: Kris Winer
date: May 1, 2014
updated: 2018 Simon D. Levy
license: Beerware - Use this code however you'd like. If you
find it useful you can buy me a beer some time.
*/

#include <stdint.h>
#include <stdbool.h>

#include "MPU.h"

class MPU60x0 : public MPUIMU {

    private:

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
        const uint8_t XG_OFFS_USRH     		= 0x13;  // User-defined trim values for gyroscope; supported in MPU-60x0?
        const uint8_t XG_OFFS_USRL     		= 0x14;
        const uint8_t YG_OFFS_USRH     		= 0x15;
        const uint8_t YG_OFFS_USRL     		= 0x16;
        const uint8_t ZG_OFFS_USRH     		= 0x17;
        const uint8_t ZG_OFFS_USRL     		= 0x18;
        const uint8_t SMPLRT_DIV       		= 0x19;
        const uint8_t CONFIG           		= 0x1A;
        const uint8_t GYRO_CONFIG      		= 0x1B;
        const uint8_t ACCEL_CONFIG     		= 0x1C;
        const uint8_t FF_THR           		= 0x1D;  // Free-fall
        const uint8_t FF_DUR           		= 0x1E;  // Free-fall
        const uint8_t MOT_THR          		= 0x1F;  // Motion detection threshold bits [7:0]
        const uint8_t MOT_DUR          		= 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
        const uint8_t ZMOT_THR         		= 0x21;  // Zero-motion detection threshold bits [7:0]
        const uint8_t ZRMOT_DUR        		= 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
        Ascale_t _aScale;
        Gscale_t _gScale;
        float    _aRes;
        float    _gRes;
        float    _accelBias[3];
        float    _gyroBias[3];

        void     calibrate(float accelBias[3], float gyroBias[3]);
        void     init(Ascale_t ascale, Gscale_t gscale);
        float    getGres(Gscale_t gscale);
        float    getAres(Ascale_t ascale);
        uint8_t  getId(void);
        void     selfTest(float * destination);

        uint8_t  readRegister(uint8_t subAddress);

        // Cross-platform support
        virtual void     writeRegister(uint8_t subAddress, uint8_t data) = 0;
        virtual void     readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) = 0;

    protected:

        const uint8_t MPU60x0_ADDRESS = 0x68;

        MPU60x0(Ascale_t ascale, Gscale_t gscale);

        MPU_Error_t begin(void);

    public:

        bool        checkNewData(void);
        void        lowPowerAccelOnly(void);

        void        readAccelerometer(float & ax, float & ay, float & az);
        void        readGyrometer(float & gx, float & gy, float & gz);
        float       readTemperature(void);

}; // class MPU60x0
