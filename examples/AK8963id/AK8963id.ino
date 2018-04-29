#include <Wire.h>

#include "ArduinoTransfer.h"

ArduinoWire bt;


// MPU9250 registers
const uint8_t EXT_SENS_DATA_00 = 0x49;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;

// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_WHO_AM_I = 0x00;

static uint8_t _buffer[21];

static void readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    Wire.beginTransmission(address); // open the device
    Wire.write(subAddress); // specify the starting register address
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, count); // specify the number of bytes to receive
    for(uint8_t i = 0; i < count; i++){ 
        dest[i] = Wire.read();
    }
}

static void writeRegister(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address); // open the device
    Wire.write(subAddress); // write the register address
    Wire.write(data); // write the data
    Wire.endTransmission();
}

/* reads registers from the AK8963 */
static void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    // set slave 0 to the AK8963 and set for read
    writeRegister(0x68, I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG);

    // set the register to the desired AK8963 sub address
    writeRegister(0x68, I2C_SLV0_REG,subAddress);

    // enable I2C and request the bytes
    writeRegister(0x68, I2C_SLV0_CTRL,I2C_SLV0_EN | count);

    delay(1); // takes some time for these registers to fill

    // read the bytes off the MPU9250 EXT_SENS_DATA registers
    readRegisters(0x68, EXT_SENS_DATA_00, count,dest); 
}

static uint8_t whoAmIAK8963()
{
    // read the WHO AM I register
    readAK8963Registers(AK8963_WHO_AM_I,1,_buffer);

    // return the register value
    return _buffer[0];
}

void setup(void)
{
    Serial.begin(115200);

    Wire.begin();

    Wire.setClock(400000);

    uint8_t addr = whoAmIAK8963();

    while (true) {
        Serial.print("0x");
        Serial.println(addr, HEX);
    }
}

void loop(void)
{
}
