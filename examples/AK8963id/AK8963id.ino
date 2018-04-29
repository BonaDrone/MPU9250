#include <Wire.h>


// MPU9250 registers
const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;

const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;

const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;

const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t ACCEL_DLPF_184 = 0x01;
const uint8_t ACCEL_DLPF_92 = 0x02;
const uint8_t ACCEL_DLPF_41 = 0x03;
const uint8_t ACCEL_DLPF_20 = 0x04;
const uint8_t ACCEL_DLPF_10 = 0x05;
const uint8_t ACCEL_DLPF_5 = 0x06;

const uint8_t CONFIG = 0x1A;
const uint8_t GYRO_DLPF_184 = 0x01;
const uint8_t GYRO_DLPF_92 = 0x02;
const uint8_t GYRO_DLPF_41 = 0x03;
const uint8_t GYRO_DLPF_20 = 0x04;
const uint8_t GYRO_DLPF_10 = 0x05;
const uint8_t GYRO_DLPF_5 = 0x06;

const uint8_t SMPDIV = 0x19;

const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_RAW_RDY_EN = 0x01;

const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;

const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;

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

const uint8_t WHO_AM_I = 0x75;

// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;

const uint8_t AK8963_HXL = 0x03; 

const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;
const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;

const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;

const uint8_t AK8963_ASA = 0x10;

const uint8_t AK8963_WHO_AM_I = 0x00;

static uint8_t _buffer[21];

uint8_t addr = 0x00;

static int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
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

    return 1;
}

/* reads registers from the AK8963 */
static void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    // set slave 0 to the AK8963 and set for read
    writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG);

    // set the register to the desired AK8963 sub address
    writeRegister(I2C_SLV0_REG,subAddress);

    // enable I2C and request the bytes
    writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count);

    delay(1); // takes some time for these registers to fill

    // read the bytes off the MPU9250 EXT_SENS_DATA registers
    readRegisters(EXT_SENS_DATA_00,count,dest); 
}

static int whoAmIAK8963()
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

    addr = whoAmIAK8963();
}

void loop(void)
{
    Serial.print("0x");
    Serial.println(addr, HEX);
}
