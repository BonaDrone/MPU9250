/*  Header for Arduino implmentations of ByteTransfer class
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "ByteTransfer.h"

class ArduinoTransfer : public ByteTransfer {

    public:

        void delayMsec(unsigned long  msec) override;
};

class ArduinoWire : public ArduinoTransfer {

	public:

		void    writeByte(uint8_t address, uint8_t subAddress, uint8_t data) override;

		uint8_t readByte(uint8_t address, uint8_t subAddress) override;

		void    readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) override;
};


class ArduinoSPI : public ArduinoTransfer {

	public:

		void    writeByte(uint8_t address, uint8_t subAddress, uint8_t data) override;

		uint8_t readByte(uint8_t address, uint8_t subAddress) override;

		void    readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) override;
};





