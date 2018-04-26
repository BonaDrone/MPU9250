/*  Header for WiringPi implmentations of ByteTransfer class
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "ByteTransfer.h"

class WiringPiI2C : public ByteTransfer {

	public:

		void    writeByte(uint8_t address, uint8_t subAddress, uint8_t data) override;

		uint8_t readByte(uint8_t address, uint8_t subAddress) override;

		void    readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) override;
};


class WiringPiSPI : public ByteTransfer {

    private:

        uint8_t _bus;
        uint32_t _speed;

	public:

                WiringPiSPI(uint8_t bus);

		void    writeByte(uint8_t address, uint8_t subAddress, uint8_t data) override;

		uint8_t readByte(uint8_t address, uint8_t subAddress) override;

		void    readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) override;
};





