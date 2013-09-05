#include <ros/ros.h>
#include <Arduino.h>
#include "wiring_private.h"
#include "SR04.h"

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "test");
  ros::NodeHandle node;
init();
int led_pin = 7;
int echo_pin = 5;
int trig_pin = 6;
SR04 sr04 = SR04( echo_pin ,trig_pin);

bool ledStatus = true;
pinMode(led_pin, OUTPUT);
 ros::Rate loopRate(2);

while (ros::ok())
{
	if (ledStatus)
	{
	digitalWrite(led_pin, HIGH);  // set the LED on
	ledStatus = false;
	}
	else
	{
	  digitalWrite(led_pin, LOW);       // set the LED off
	ledStatus = true;
	}
	  ROS_INFO("test ok");
ROS_INFO("sensor: %d cm", sr04.Distance());
	ros::spinOnce();
	loopRate.sleep();
}

return 0;
}

