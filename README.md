# ROS Arduino Budget INS

This is a ROSSerial Arduino driver for the [SparkFun IMU ICM-20948](https://www.sparkfun.com/products/15335), [SparkFun IMU BNO080](https://www.sparkfun.com/products/14686), and [Arduino Nano33 Sense BLE](https://store.arduino.cc/usa/nano-33-ble-sense)  The total cost of these three items is about $83.  The full sensor list supported by this library:

* ICM-20948: 9DOF IMU with On-board Fusion
* BNO080: 9DOF IMU with On-board Fusion
* LSM9DS1: 9DOF IMU
* HTS221: Temperature and Humidity Sensor
* LPS22HB: Pressure Sensor

It is tested with ROS1 Noetic on a Raspberry Pi 4B with an Arduino Nano33 Sense BLE but may work on other configurations.

## Repository Contents

* [**/examples**](./examples) - Example sketches for the library (.ino). Run from the Arduino IDE.
* [**/main_driver**](./main_driver) - Main sketch for the library (.ino). Run from the Arduino IDE.
* [**/src**](./src) - Source files for the ICM-20948 library (.cpp, .h).
* [**CONTRIBUTING.md**](./CONTRIBUTING.md) - Guidelines on how to contribute to this library.

## Documentation

A lot of information and test scripts have been removed from this library for simplicity, refer to the original libraries for more information.

* **[Arduino LSM9DS1 Library](https://github.com/arduino-libraries/Arduino_LSM9DS1)**
* **[Arduino HTS221 Library](https://github.com/arduino-libraries/Arduino_HTS221)**
* **[Arduino LPS22HB Library](https://github.com/arduino-libraries/Arduino_LPS22HB)**
* **[Sparkfun BNO080 Library](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library)**
* **[Sparkfun ICM-20948 Library](https://github.com/GAVLab/ros_arduino_ICM-20948_BNO080)** (included in this repository)

## Installation

* Install the libraries above by searching in the Arduino IDE `Tools > Manage Libraries...`.  You do not need to install ICM-20948 which has been included and modified to support the Digital Motion Processor (DMP).
* Install ROS Serial.  Change noetic to your ROS version: kinetic, melodic, noetic, etc.
```
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```
* Install ros_lib into the Arduino Environment
```
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

## License Information

### Sparkfun Libraries

This product is _**open source**_!

Please see the included [License.md](./License.md) for more information.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.

### Arduino Libraries

Copyright (c) 2019 Arduino SA. All rights reserved.

This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
