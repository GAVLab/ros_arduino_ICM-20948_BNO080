/*
  Arduino LSM9DS1 - Simple IMU

  This example reads the acceleration, gyroscope, and magnometer values from the LSM9DS1
  sensor and continuously prints them to the SERIAL_PORT Monitor
  or SERIAL_PORT Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include <Arduino_LSM9DS1.h>

#define GACC 9.80665f // Acceleration due to gravity on Earth
#define SERIAL_PORT Serial

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT);
  SERIAL_PORT.println("Started");

  if (!IMU.begin()) {
    SERIAL_PORT.println("Failed to initialize IMU!");
    while (1);
  }

  SERIAL_PORT.print("Accelerometer sample rate = ");
  SERIAL_PORT.print(IMU.accelerationSampleRate());
  SERIAL_PORT.println(" Hz");
  SERIAL_PORT.print("Gyroscope sample rate = ");
  SERIAL_PORT.print(IMU.gyroscopeSampleRate());
  SERIAL_PORT.println(" Hz");
  SERIAL_PORT.print("Magnetic field sample rate = ");
  SERIAL_PORT.print(IMU.magneticFieldSampleRate());
  SERIAL_PORT.println(" Hz");
}

void loop() {
  float gax,gay,gaz,ax,ay,az,gx,gy,gz,mx,my,mz;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(gax,gay,gaz);
    ax = gax*GACC;
    ay = gay*GACC;
    az = gaz*GACC;
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }

  SERIAL_PORT.print(F("(ax:"));
  SERIAL_PORT.print(ax,2);
  SERIAL_PORT.print(F(",ay:"));
  SERIAL_PORT.print(ay,2);
  SERIAL_PORT.print(F(",az:"));
  SERIAL_PORT.print(az,2);
  SERIAL_PORT.print(F(")m/s^2\t"));
  
  SERIAL_PORT.print(F("(gx:"));
  SERIAL_PORT.print(gx,2);
  SERIAL_PORT.print(F(",gy:"));
  SERIAL_PORT.print(gy,2);
  SERIAL_PORT.print(F(",gz:"));
  SERIAL_PORT.print(gz,2);
  SERIAL_PORT.print(F(")degrees/s\t")); 

  SERIAL_PORT.print(F("(mx:"));
  SERIAL_PORT.print(mx,2);
  SERIAL_PORT.print(F(",my:"));
  SERIAL_PORT.print(my,2);
  SERIAL_PORT.print(F(",mz:"));
  SERIAL_PORT.print(mz,2);
  SERIAL_PORT.println(F(")uT\t")); 

}
