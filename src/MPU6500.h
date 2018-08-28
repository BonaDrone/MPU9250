/*  Header file for MPU6500 class library
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "MPU.h"

class MPU6500 : public MPUIMU {

    public:

        MPU6500(Ascale_t ascale, Gscale_t gscale);

        MPU_Error_t begin(void);

        bool        checkNewData(void);

        void        readAccelerometer(float & ax, float & ay, float & az);

        void        readGyrometer(float & gx, float & gy, float & gz);

    protected:

        virtual void writeMPURegister(uint8_t subAddress, uint8_t data) override;

        virtual void readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;

        virtual uint8_t xAOffsetH(void) override;
        virtual uint8_t yAOffsetH(void) override;
        virtual uint8_t zAOffsetH(void) override;
}; 
