/* MPU6050 library header code
by: Kris Winer
date: May 1, 2014
updated: 2018 Simon D. Levy
license: Beerware - Use this code however you'd like. If you
find it useful you can buy me a beer some time.
*/

#include "MPU60x0.h"

class MPU6050 : public MPU60x0 {

    public:

        MPU6050(Ascale_t ascale, Gscale_t gscale);

        MPU_Error_t begin(uint8_t bus=1);

    private:

        uint8_t  _i2c;

        virtual void writeRegister(uint8_t subAddress, uint8_t data)override;
        virtual void readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)override;
}; 
