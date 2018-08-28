This repository comes from a few different source files created by [Kris Winer](https://github.com/kriswiner) 
to support the Invensense MPU line of Internal Measurement Unit (IMU) sensors.  

Here's what I changed:
* Reorganized Kris's
[source code](https://github.com/kriswiner/MPU9250/tree/master/MPU9250_BME280_SPIFlash_Ladybug) 
into the modern Arduino library [format](https://github.com/arduino/arduino/wiki/arduino-ide-1.5:-library-specification)
* Simplified the API
* Added support for RaspberryPi via [WiringPi](http://wiringpi.com/)
* Added support for NVIDIA Jetson via i2cdev
* Simplified the 
[examples](https://github.com/kriswiner/MPU9250/blob/master/MPU9250_BME280_SPIFlash_Ladybug/MPU9250_BME280_SPIFlash_Ladybug.ino) 
by removing code requiring components other than the MPU sensors

The following sensors are currently supported:

* MPU6050
* MPU9250
* MPU6500

The MPU6050 supports communication via the I<sup>2</sup>C bus.To use this bus, you should also clone the
[CrossPlatformI2C](https://github.com/simondlevy/CrossPlatformI2C) repository.

The MPU6500 supports communication via the SPI bus.To use this bus, you should also clone the
[CrossPlatformSPI](https://github.com/simondlevy/CrossPlatformSPI) repository.

The MPU9250 supports communication via either bus.  With I<sup>2</sup>C you can run the
MPU9250 in &ldquo;master mode&rdquo;, meaning that you access both sensors
(MPU6500 accelerometer/gyrometer, AK8963 magnetometer) at once, or in &ldquo;pass-through
mode&rdquo; allowing you to communicate directly with the magnetometer for more efficient data acquisition.  
With the SPI bus you get only master mode.  To use the I<sup>2</sup>C bus, you should also clone the
[CrossPlatformI2C](https://github.com/simondlevy/CrossPlatformI2C) repository.
To use the SPI bus, you should also clone the
[CrossPlatformSPI](https://github.com/simondlevy/CrossPlatformSPI) repository.  

I have tested this library on the following hardware:

* [Butterfly STM32L433](https://www.tindie.com/products/TleraCorp/butterfly-stm32l433-development-board/) 
development board, I<sup>2</sup>C

* Teensy 3.2 development board, I<sup>2</sup>C

* Raspberry Pi 3, I<sup>2</sup>C

* Raspberry Pi Zero with [PXFMini](http://erlerobotics.com/blog/product/pxfmini/) autopilot shield, SPI 

* NVIDIA Jetson TX1, I<sup>2</sup>C (fails tolerance self-test)

RaspberryPi users should download and install
[WiringPi](http://wiringpi.com/), then cd to one of the example folders in 
<b>MPU9250/extras/wiringpi/examples</b>, and type <tt>sudo make run</tt>.

NVIDIA Jetson users should install I<sup>2</sup>C support by running the command:
<pre>
  sudo apt-get install libi2c-dev i2c-tools
</pre>
