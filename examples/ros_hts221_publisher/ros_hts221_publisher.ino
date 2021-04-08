/*
 * rosserial Publisher Example
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <Arduino_HTS221.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

std_msgs::Float32 temp_msg;
std_msgs::Float32 humidity_msg;

ros::Publisher pub_temp("hts221/temperature", &temp_msg);
ros::Publisher pub_humidity("hts221/humidity", &humidity_msg);
long hts221_timer;


// Frequency rate in Hz
unsigned int hts221_rate = 1;

void setup()
{
  delay(50);
  nh.initNode();
  nh.advertise(pub_temp);
  nh.advertise(pub_humidity);
  delay(50);
  if (!HTS.begin()) {
    //Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
  hts221_timer = millis();
}

void loop()
{
  if (millis() - hts221_timer > (1000/hts221_rate)){ 
    hts221_timer = millis();
    float temperature = HTS.readTemperature();
    float humidity    = HTS.readHumidity()/100.0;
    
    temp_msg.data = temperature;
    humidity_msg.data = humidity;
    
    pub_temp.publish(&temp_msg);
    pub_humidity.publish(&humidity_msg);
    nh.spinOnce();
  }
  delay(1);
}
