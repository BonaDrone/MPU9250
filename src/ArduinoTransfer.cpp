/*  Code for Arduino implmentations of ByteTransfer class
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "ArduinoTransfer.h"

#include <Arduino.h>

#if defined(__MK20DX256__)  
#include <i2c_t3.h>   
#else
#include <Wire.h>   
#endif

void ArduinoI2C::writeRegister(uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(_address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

void ArduinoI2C::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(_address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(_address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read();          // Put read results in the Rx buffer
  }
}

          
          

