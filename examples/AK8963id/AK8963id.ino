#include <Wire.h>

// Define I2C addresses of MPU9250
#define ADO 0
#if ADO
static const uint8_t MPU9250_ADDRESS  = 0x69;   // Device address when ADO = 1
#else
static const uint8_t MPU9250_ADDRESS  = 0x68;  // Device address when ADO = 0
#endif  

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

static void readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest){
    Wire.beginTransmission(address); // open the device
    Wire.write(subAddress); // specify the starting register address
    Wire.endTransmission(false);
    Wire.requestFrom(address, count); // specify the number of bytes to receive
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
    writeRegister(MPU9250_ADDRESS, I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG);

    // set the register to the desired AK8963 sub address
    writeRegister(MPU9250_ADDRESS, I2C_SLV0_REG,subAddress);

    // enable I2C and request the bytes
    writeRegister(MPU9250_ADDRESS, I2C_SLV0_CTRL,I2C_SLV0_EN | count);

    delay(1); // takes some time for these registers to fill

    // read the bytes off the MPU9250 EXT_SENS_DATA registers
    readRegisters(MPU9250_ADDRESS, EXT_SENS_DATA_00,count,dest); 
}

/* writes a register to the AK8963 given a register address and data */
static void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    // set slave 0 to the AK8963 and set for write
    writeRegister(MPU9250_ADDRESS, I2C_SLV0_ADDR,AK8963_I2C_ADDR);

    // set the register to the desired AK8963 sub address 
    writeRegister(MPU9250_ADDRESS, I2C_SLV0_REG,subAddress);

    // store the data for write
    writeRegister(MPU9250_ADDRESS, I2C_SLV0_DO,data);

    // enable I2C and send 1 byte
    writeRegister(MPU9250_ADDRESS, I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1);

    // pause briefly to complete writing
    delay(1);
}

static int whoAmIAK8963()
{
    uint8_t buffer = 0;

    // read the WHO AM I register
    readAK8963Registers(AK8963_WHO_AM_I,1, &buffer);

    // return the register value
    return buffer;
}

void setup(void)
{
    Serial.begin(115200);

    Wire.begin();

    Wire.setClock(400000);

    // select clock source to gyro
    writeRegister(MPU9250_ADDRESS, PWR_MGMNT_1,CLOCK_SEL_PLL);

    // enable I2C master mode
    writeRegister(MPU9250_ADDRESS, USER_CTRL,I2C_MST_EN);

    // set the I2C bus speed to 400 kHz
    writeRegister(MPU9250_ADDRESS, I2C_MST_CTRL,I2C_MST_CLK);

    // set AK8963 to Power Down
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

    // reset the MPU9250
    writeRegister(MPU9250_ADDRESS, PWR_MGMNT_1,PWR_RESET);

    // wait for MPU-9250 to come back up
    delay(1);

    // reset the AK8963
    writeAK8963Register(AK8963_CNTL2,AK8963_RESET);

    // select clock source to gyro
    writeRegister(MPU9250_ADDRESS, PWR_MGMNT_1,CLOCK_SEL_PLL);

    // enable I2C master mode
    writeRegister(MPU9250_ADDRESS, USER_CTRL,I2C_MST_EN);

    // set the I2C bus speed to 400 kHz
    writeRegister(MPU9250_ADDRESS, I2C_MST_CTRL,I2C_MST_CLK);

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
