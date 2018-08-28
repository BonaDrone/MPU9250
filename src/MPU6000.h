/*  Header file for MPU6000 class library
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

class MPU6000 : public MPU60x0 {

    public:

        MPU6000(Ascale_t ascale, Gscale_t gscale);

        MPU_Error_t begin(void);

    protected:

        virtual void writeMPURegister(uint8_t subAddress, uint8_t data) override;

        virtual void readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;
}; 
