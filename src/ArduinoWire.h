/*  Header for Arduino Wire implmentation of ByteTransfer class
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "ByteTransfer.h"

class ArduinoWire : public ByteTransfer {

	public:

		void    writeByte(uint8_t address, uint8_t subAddress, uint8_t data) override;

		uint8_t readByte(uint8_t address, uint8_t subAddress) override;

		void    readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) override;
};




