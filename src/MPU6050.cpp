/* MPU6050 implementation code
by: Kris Winer
date: May 1, 2014
updated: August 2018 Simon D. Levy
license: Beerware - Use this code however you'd like. If you
find it useful you can buy me a beer some time.
 */

#include "MPU6050.h"
#include "CrossPlatformI2C_Core.h"

MPU6050::MPU6050(Ascale_t ascale, Gscale_t gscale) : MPU60x0(ascale, gscale)
{
}

MPU_Error_t MPU6050::begin(uint8_t bus)
{
    _i2c = cpi2c_open(MPU60x0_ADDRESS, bus);

    return MPU60x0::begin();
}

void MPU6050::writeRegister(uint8_t subAddress, uint8_t data)
{
    cpi2c_writeRegister(_i2c, subAddress, data);
}

void MPU6050::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    cpi2c_readRegisters(_i2c, subAddress, count, dest);
}
