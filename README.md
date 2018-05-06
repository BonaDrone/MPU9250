# MPU9250

This repository comes from a few different source files created by [Kris Winer](https://github.com/kriswiner) 
to support the MPU9250 9-DOF Internal Measurement Unit.  

Here's what I changed:
* Reorganized Kris's
[source code](https://github.com/kriswiner/MPU9250/tree/master/MPU9250_BME280_SPIFlash_Ladybug) 
into the modern Arduino library format 
* Added support for RaspberryPi via [WiringPi](http://wiringpi.com/)
* Simplified the 
[examples](https://github.com/kriswiner/MPU9250/blob/master/MPU9250_BME280_SPIFlash_Ladybug/MPU9250_BME280_SPIFlash_Ladybug.ino) 
by removing code requiring components other than the MPU9250
