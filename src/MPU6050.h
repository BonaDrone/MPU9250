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

#include "MPU60x0.h"

class MPU6050 : public MPU60x0 {

    public:

        MPU6050(Ascale_t ascale, Gscale_t gscale);

        MPU_Error_t begin(uint8_t bus=1);

    protected:

            virtual void writeMPURegister(uint8_t subAddress, uint8_t data) override;

            virtual void readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;

    private:

            // Cross-platform support
            uint8_t  _i2c;
}; 
