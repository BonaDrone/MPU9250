/* MPU6000 implementation code
by: Kris Winer
date: May 1, 2014
updated: August 2018 Simon D. Levy
license: Beerware - Use this code however you'd like. If you
find it useful you can buy me a beer some time.
 */

#include "MPU6000.h"
#include "CrossPlatformSPI.h"

MPU6000::MPU6000(Ascale_t ascale, Gscale_t gscale) : MPU60x0(ascale, gscale)
{
}

MPU_Error_t MPU6000::begin(void)
{
    return MPU60x0::begin();
}


void MPU6000::writeRegister(uint8_t subAddress, uint8_t data)
{
    cpspi_writeRegister(subAddress, data);
}

void MPU6000::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    return cpspi_readRegisters(subAddress, count, dest);
}
