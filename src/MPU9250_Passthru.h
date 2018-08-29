/*  Header file for pass-through MPU9250 class
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

#include "MPU9250.h"

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
