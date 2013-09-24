#include <ros/ros.h>
#include <Arduino.h>
#include "wiring_private.h"
#include "SR04.h"
#include <list>
#include <sensor_msgs/JointState.h>
#include "MyNodeHandle.h"
#include "Servo32.h"

Servo32 servos;
void jointCallback(const sensor_msgs::JointStateConstPtr& msg)
{/*
  for (int i = 0; i < msg->name.size(); i++)
  {
    servos.nameMap.insert(std::pair<std::string, int>(msg->name[i], i));
  }
  ROS_INFO("joint...");
  servos.run(msg->name, msg->position, msg->velocity);*/
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "pantilt");
  MyNodeHandle node;
  ros::Subscriber goalVelSub = node.subscribe("/joint_states", 1, jointCallback);
  sensor_msgs::JointState joint;

  ros::spin();

  return 0;
}
