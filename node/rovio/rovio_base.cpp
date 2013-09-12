#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "MyNodeHandle.h"
#include "Motor.h"

// params
Motor motorA;
Motor motorB;
Motor motorC;
double rateX, rateY, rateRear, rateFront;
double maxLinear, maxAngular, axialLenRate;

void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
  float rotA = 0, rotB = 0, rotC = 0;
  double tmp;

  // x
  tmp = msg->linear.x * rateX;
  rotA -= tmp;
  rotB += tmp;

  // y
  tmp = msg->linear.y * rateY;
  rotA += tmp;
  rotB += tmp;
  rotC -= tmp / axialLenRate;

  // rotate
  tmp = msg->angular.z * rateRear;
  rotC += tmp;
  tmp *= axialLenRate;
  rotA += tmp;
  rotB += tmp;

  motorA.run(rotA);
  motorB.run(rotB);
  motorC.run(rotC);
  ROS_INFO("Motor: rotA[%.4f] rotB[%.4f] rotC[%.4f]", rotA, rotB, rotC);
}


int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "rovio_base");
  MyNodeHandle node;
  ros::Subscriber cmdVelSub = node.subscribe("/cmd_vel", 1, cmdVelCallback);

  // get params
  int motorAEn = node.getParamEx("rovio_base/motorAEn", 3);
  int motorA1 = node.getParamEx("rovio_base/motorA1", 4);
  int motorA2 = node.getParamEx("rovio_base/motorA2", 5);
  int motorBEn = node.getParamEx("rovio_base/motorBEnl", 6);
  int motorB1 = node.getParamEx("rovio_base/motorB1", 7);
  int motorB2 = node.getParamEx("rovio_base/motorB2", 8);
  int motorCEn = node.getParamEx("rovio_base/motorCEn", 9);
  int motorC1 = node.getParamEx("rovio_base/motorC1", 10);
  int motorC2 = node.getParamEx("rovio_base/motorC2", 11);
  double biasAngle = node.getParamEx("rovio_base/biasAngle", 0.5236); // Bias angle (radian) of the front wheels
  axialLenRate = node.getParamEx("rovio_base/axialLenRate", 0.667); // front / rear is about 2/3
  maxLinear = node.getParamEx("rovio_base/maxLinear", 0.4);
  maxAngular = node.getParamEx("rovio_base/maxAngular", 2.0);

  // init motors
  motorA.init(motorAEn, motorA1, motorA2);
  motorB.init(motorBEn, motorB1, motorB2);
  motorC.init(motorCEn, motorC1, motorC2);

  // calc rates
  rateX = 1 / (cos(biasAngle) * maxLinear);
  rateY = 1 / (sin(biasAngle) * maxLinear);
  rateRear = 1 / maxAngular;

  ros::spin();
  return 0;
}
