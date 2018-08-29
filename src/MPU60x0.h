/*  Header file for MPU60x0 class library
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

class MPU60x0 : public MPUIMU {

    public:

        MPU_Error_t begin(void);

        bool        checkNewData(void);

        void        lowPowerAccelOnly(void);

        void        readGyrometer(float & gx, float & gy, float & gz);

        float       readTemperature(void);

    protected:

        MPU60x0(Ascale_t ascale, Gscale_t gscale);

    private:

        const uint8_t SELF_TEST_X      		= 0x0D;
        const uint8_t SELF_TEST_Y      		= 0x0E;
        const uint8_t SELF_TEST_Z      		= 0x0F;
        const uint8_t SELF_TEST_A      		= 0x10;
        const uint8_t MOT_THR          		= 0x1F;  
        const uint8_t MOT_DUR          		= 0x20; 

        void     selfTest(float * tolerances);

        void     init(void);
}; 
