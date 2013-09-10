#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "MyNodeHandle.h"
#include <Arduino.h>
#include "motor.h"

void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
  moto_pwm_set(3, 1, 2);
  set_moto_turn_to(1, 2, HIGH, LOW);
}


int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "rovio_base");
  MyNodeHandle node;
  ros::Subscriber cmdVelSub = node.subscribe("/cmd_vel", 1, cmdVelCallback);

  pinMode(1,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  ros::spin();
  return 0;
}
