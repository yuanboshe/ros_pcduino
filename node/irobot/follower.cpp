#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "MyNodeHandle.h"
#include "Utility.h"

using namespace cv;

ros::Publisher cmdVelPub;

// params
bool paused; // dynamic pause or resume this program
int paramMinRange, paramMaxRange;
double paramMaxWeight;
int paramGoalZ, paramMinPoints, paramMargin;
double paramMaxLinear, paramMaxAngular;

bool stopFlag = false;
void commandCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "follower")
  {
    paused = false;
    stopFlag = true;
  }
  else
  {
    paused = true;
    if (stopFlag)
    {
      geometry_msgs::Twist moveCmd;
      moveCmd.linear.x = 0;
      moveCmd.angular.z = 0;
      cmdVelPub.publish(moveCmd);
      stopFlag = false;
    }
  }
}

float frontRange = 0;
void frontCallback(const sensor_msgs::RangeConstPtr& msg)
{
  frontRange = msg->range;
}

float leftRange = 0;
void leftCallback(const sensor_msgs::RangeConstPtr& msg)
{
  leftRange = msg->range;
}

float rightRange = 0;
void rightCallback(const sensor_msgs::RangeConstPtr& msg)
{
  rightRange = msg->range;
}

void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if (paused)
    return;
  // convert sensor_msgs/Image to Mat
  cv_bridge::CvImagePtr cvImgPtr;
  Mat_<uint16_t> depthImg;
  try
  {
    cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    depthImg = cvImgPtr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // init
  Utility util;
  int rows = depthImg.rows;
  int cols = depthImg.cols;

  try
  {
    // calc weight & center
    float range[2] = {paramMinRange, paramMaxRange};
    Mat_<float> wMat;
    Point3f center;
    center = util.calcWeightCneter(depthImg, wMat, range, paramMinPoints, paramMaxWeight, paramMargin);

    // control
    float linearBias = center.z - paramGoalZ;
    float angularBias = cols / 2 - center.x;

    float linearSpeed = 0;
    float angularSpeed = 0;
    if (center.z > 0)
    {
      linearSpeed = linearBias / (paramMaxRange - paramGoalZ) * paramMaxLinear;
      angularSpeed = angularBias * 2 / cols * paramMaxAngular;
    }

    // pub message
    geometry_msgs::Twist moveCmd;
    moveCmd.linear.x = linearSpeed;
    moveCmd.linear.y = 0;
    moveCmd.angular.z = angularSpeed;
    cmdVelPub.publish(moveCmd);
  }
  catch (Exception& e)
  {
    ROS_ERROR("follower depthCallback exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "follower");
  MyNodeHandle node;
  ros::Subscriber depthRawSub = node.subscribe("/depth/image_raw", 10, depthCallback);
  ros::Subscriber commandSub = node.subscribe("/cmd_center/author", 100, commandCallback);
  ros::Subscriber frontSub = node.subscribe("/sonar/front", 100, frontCallback);
  ros::Subscriber leftSub = node.subscribe("/sonar/left", 100, leftCallback);
  ros::Subscriber rightSub = node.subscribe("/sonar/right", 100, rightCallback);
  cmdVelPub = node.advertise<geometry_msgs::Twist>("/goal_vel", 100);

  // get params
  ROS_INFO("follower get params:");
  paused = node.getParamEx("follower/paused", true);
  paramMinRange = node.getParamEx("follower/minRange", 500);
  paramMaxRange = node.getParamEx("follower/maxRange", 1000);
  paramGoalZ = node.getParamEx("follower/goalZ", 500);
  paramMargin = node.getParamEx("follower/margin", 0);
  paramMinPoints = node.getParamEx("follower/minPoints", 2000);
  paramMaxWeight = node.getParamEx("follower/maxWeight", 8.0);
  paramMaxLinear = node.getParamEx("follower/maxLinear", 0.5);
  paramMaxAngular = node.getParamEx("follower/maxAngular", 2.0);

  ros::spin();

  return 0;
}
