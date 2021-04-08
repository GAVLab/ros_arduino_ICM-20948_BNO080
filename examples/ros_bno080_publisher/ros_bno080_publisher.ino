/*
 * rosserial Publisher Example
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

ros::NodeHandle  nh;

sensor_msgs::Imu bno080_msg;

ros::Publisher pub_bno080("bno080/data", &bno080_msg);
//long bno080_timer;

// Frequency rate in Hz
//unsigned int hts221_rate = 1;

void setup()
{
  delay(50);
  nh.initNode();
  nh.advertise(pub_bno080);
  delay(50);
  Wire.begin();
  if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(50); //Send data update every 50ms
  //bno080_timer = millis();
}

void loop()
{
  if (myIMU.dataAvailable() == true)
  {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

    bno080_msg.orientation.x = quatI;
    bno080_msg.orientation.y = quatJ;
    bno080_msg.orientation.z = quatK;
    bno080_msg.orientation.w = quatReal;
    bno080_msg.header.stamp = nh.now();
    pub_bno080.publish(&bno080_msg);
    
    nh.spinOnce();
  }
  delay(1);
}
