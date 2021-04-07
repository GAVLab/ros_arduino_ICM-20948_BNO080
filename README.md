# ROS Arduino ICM-20948 BNO080

This is a ROSSerial Arduino driver with the SparkFun libraries for the TDK InvenSense ICM-20948 Inertial Measurement Unit 9-Degree Of Freedom sensor as used on the [SparkFun IMU ICM-20948](https://www.sparkfun.com/products/15335) and the BNO080 Inertial Measurement Unit 9-Degree of Freedom sensor as used on the [SparkFun IMU BNO080](https://www.sparkfun.com/products/14686).

Both of these IMUs have a built in filter with autocalibration to produce a rotation vector in quaternions.  This driver extracts those rotation vectors and sends them via USB to the ROS master.  The raw IMU data is omitted to prevent timing issues.  All of the communication is done via I2C between the IMUs and the Arduino but it could be reconfigured for SPI.  This was tested on an Arduino Sense Nano.

## Repository Contents

* [**/examples**](./examples) - Example sketches for the library (.ino). Run these from the Arduino IDE.
* [**/src**](./src) - Source files for the library (.cpp, .h).
* [**CONTRIBUTING.md**](./CONTRIBUTING.md) - Guidelines on how to contribute to this library.

## Documentation

A lot of information and test scripts have been removed from this library for simplicity, refer to the original libraries for more information.

* **[Sparkfun ICM-20948 Library](https://github.com/GAVLab/ros_arduino_ICM-20948_BNO080)**
* **[Sparkfun BNO080 Library](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library)**

## Installation

* Install the library for BNO080 by searching `SparkFun_BNO080` in the Arduino IDE `Tools > Manage Libraries...`.  You do not need to install ICM-20948 which has been included and modified to support the Digital Motion Processor (DMP).

## License Information

This product is _**open source**_!

Please see the included [License.md](./License.md) for more information.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
