/*  Header for Arduino implmentations of ByteTransfer class
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "ByteTransfer.h"

class ArduinoI2C : public I2CTransfer {

	public:

        ArduinoI2C(uint8_t address) : I2CTransfer(address) { }

		void    writeRegister(uint8_t subAddress, uint8_t data) override;
		void    readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;
};
