/* 
   MPU60x0.h: Header MPU60x0 classes

   Copyright (C) 2018 Simon D. Levy

   Adapted from https://github.com/kriswiner/MPU6050/blob/master/MPU6050BasicExample.ino

   This file is part of MPU.

   MPU is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   MPU is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with MPU.  If not, see <http://www.gnu.org/licenses/>.
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
