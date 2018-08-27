/* MPU/6000 library header code
by: Kris Winer
date: May 1, 2014
updated: 2018 Simon D. Levy
license: Beerware - Use this code however you'd like. If you
find it useful you can buy me a beer some time.
*/

#include "MPU60x0.h"

class MPU6000 : public MPU60x0 {

    public:

        MPU6000(Ascale_t ascale, Gscale_t gscale);

        MPU_Error_t begin(void);

    private:

        virtual void writeRegister(uint8_t subAddress, uint8_t data)override;
        virtual void readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)override;
};
