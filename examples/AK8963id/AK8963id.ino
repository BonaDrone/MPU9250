#include <Wire.h>

// MPU9250 registers
const uint8_t EXT_SENS_DATA_00 = 0x49;

const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;

const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D;
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;

// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;

const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;

const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;

const uint8_t AK8963_WHO_AM_I = 0x00;

static uint8_t _buffer[21];

static void error(int8_t e)
{
    while (true) {
        Serial.print("Error: ");
        Serial.println(e);
    }
}

static int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
    Wire.beginTransmission(0x68); // open the device
    Wire.write(subAddress); // specify the starting register address
    Wire.endTransmission(false);
    int _numBytes = Wire.requestFrom(0x68, count); // specify the number of bytes to receive
    if (_numBytes == count) {
        for(uint8_t i = 0; i < count; i++){ 
            dest[i] = Wire.read();
        }
        return 1;
    } else {
        return -1;
    }
}

static int writeRegister(uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(0x68); // open the device
    Wire.write(subAddress); // write the register address
    Wire.write(data); // write the data
    Wire.endTransmission();

    delay(10);

    /* read back the register */
    readRegisters(subAddress,1,_buffer);
    /* check the read back register against the written register */
    if(_buffer[0] == data) {
        return 1;
    }
    else{
        return -1;
    }
}

/* reads registers from the AK8963 */
static int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
    // set slave 0 to the AK8963 and set for read
    if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) < 0) {
        return -1;
    }
    // set the register to the desired AK8963 sub address
    if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
        return -2;
    }
    // enable I2C and request the bytes
    if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count) < 0) {
        return -3;
    }
    delay(1); // takes some time for these registers to fill
    // read the bytes off the MPU9250 EXT_SENS_DATA registers
    int _status = readRegisters(EXT_SENS_DATA_00,count,dest); 
    return _status;
}

/* writes a register to the AK8963 given a register address and data */
static int writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    // set slave 0 to the AK8963 and set for write
    if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR) < 0) {
        return -1;
    }
    // set the register to the desired AK8963 sub address 
    if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
        return -2;
    }
    // store the data for write
    if (writeRegister(I2C_SLV0_DO,data) < 0) {
        return -3;
    }
    // enable I2C and send 1 byte
    if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1) < 0) {
        return -4;
    }
    // read the register and confirm
    if (readAK8963Registers(subAddress,1,_buffer) < 0) {
        return -5;
    }
    if(_buffer[0] == data) {
        return 1;
    } else{
        return -6;
    }
}

static int whoAmIAK8963()
{
    // read the WHO AM I register
    if (readAK8963Registers(AK8963_WHO_AM_I,1,_buffer) < 0) {
        return -1;
    }

    // return the register value
    return _buffer[0];
}

void setup(void)
{
    Serial.begin(115200);

    Wire.begin();

    Wire.setClock(400000);

    // select clock source to gyro
    if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
        error(1);
    }

    // enable I2C master mode
    if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
        error(2);
    }

    // set the I2C bus speed to 400 kHz
    if(writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
        error(3);
    }

    // set AK8963 to Power Down
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

    // reset the MPU9250
    writeRegister(PWR_MGMNT_1,PWR_RESET);

    // wait for MPU-9250 to come back up
    delay(1);

    // reset the AK8963
    writeAK8963Register(AK8963_CNTL2,AK8963_RESET);

    // select clock source to gyro
    if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
        error(4);
    }

    // enable I2C master mode
    if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
        error(12);
    }

    // set the I2C bus speed to 400 kHz
    if( writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
        error(13);
    }

    // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
    uint8_t addr = whoAmIAK8963();

    while (true) {
        Serial.print("0x");
        Serial.println(addr, HEX);
    }
}

void loop(void)
{
}
