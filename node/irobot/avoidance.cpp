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
bool paused; // dynamic pause or resume this program
double maxLinear;
double maxAngular;
int goalRotationDepth;
int goalFrontDepth;
int minRotationPoints;
int maxRotationDepth;
int minFrontPoints;
int maxFrontDepth;
int frontWidth;
int frontHeight;
int rotationWidth;
int rotationHeight;
bool isSonarOn;
double minSonarFront;

double linearSpeed = 0;
double angularSpeed = 0;

bool stopFlag = false;
void commandCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "avoidance")
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
    // calc obstacle front
    int frontX = (cols - frontWidth) / 2;
    int frontY = rows - frontHeight;
    Rect frontRect = Rect(frontX, frontY, frontWidth, frontHeight);
    Mat frontRoi = depthImg(frontRect);

    // calc obstacle left right
    int rotationY = rows - rotationHeight;
    int rightX = cols - rotationWidth;
    Rect leftRect = Rect(0, rotationY, rotationWidth, rotationHeight);
    Rect rightRect = Rect(rightX, rotationY, rotationWidth, rotationHeight);
    Mat leftRoi = depthImg(leftRect);
    Mat rightRoi = depthImg(rightRect);

    // calc front info
    double avFrontDepth = maxFrontDepth;
    int frontPointSum = 0;
    double frontDepthSum = 0;
    for (int i = 0; i < frontRoi.rows; i++)
    {
      uint16_t* pRow = frontRoi.ptr<uint16_t>(i);
      for (int j = 0; j < frontRoi.cols; j++)
      {
        if (pRow[j] < maxFrontDepth)
        {
          frontPointSum++;
          frontDepthSum += pRow[j];
        }
      }
    }
    if (frontPointSum > minFrontPoints)
    {
      avFrontDepth = frontDepthSum / frontPointSum;
    }

    // calc rotation info
    double avLeftDepth = maxRotationDepth;
    double avRightDepth = maxRotationDepth;
    int leftPointSum = 0;
    int rightPointSum = 0;
    double leftDepthSum = 0;
    double rightDepthSum = 0;
    for (int i = 0; i < leftRoi.rows; i++)
    {
      uint16_t* pRow = leftRoi.ptr<uint16_t>(i);
      for (int j = 0; j < leftRoi.cols; j++)
      {
        if (pRow[j] < maxRotationDepth)
        {
          leftPointSum++;
          leftDepthSum += pRow[j];
        }
      }
    }
    for (int i = 0; i < rightRoi.rows; i++)
    {
      uint16_t* pRow = rightRoi.ptr<uint16_t>(i);
      for (int j = 0; j < rightRoi.cols; j++)
      {
        if (pRow[j] < maxRotationDepth)
        {
          rightPointSum++;
          rightDepthSum += pRow[j];
        }
      }
    }
    if (leftPointSum > minRotationPoints)
    {
      avLeftDepth = leftDepthSum / leftPointSum;
    }
    if (rightPointSum > minRotationPoints)
    {
      avRightDepth = rightDepthSum / rightPointSum;
    }

    // control
    double linearBiasDepth = avFrontDepth - goalFrontDepth;
    linearSpeed = linearBiasDepth > 0 ? (linearBiasDepth / maxFrontDepth) * maxLinear : linearBiasDepth / goalFrontDepth * maxLinear;
    if (linearBiasDepth > 0.05)
    {
      if (avLeftDepth < avRightDepth)
      angularSpeed = -(1 - avLeftDepth / maxRotationDepth) * maxAngular;
      else angularSpeed = (1 - avRightDepth / maxRotationDepth) * maxAngular;
    }
    else if (avLeftDepth < goalRotationDepth)
    {
      linearSpeed = 0;
      angularSpeed = -0.05;
    }
    else if (avRightDepth < goalRotationDepth)
    {
      linearSpeed = 0;
      angularSpeed = 0.05;
    }
    else
    {
      linearSpeed = 0;
      angularSpeed = 0;
    }


    // pub message
    geometry_msgs::Twist moveCmd;
    moveCmd.linear.x = linearSpeed;
    moveCmd.angular.z = angularSpeed;
    cmdVelPub.publish(moveCmd);
  }
  catch (Exception& e)
  {
    ROS_ERROR("obstacle avoidance depthCallback exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "avoidance");
  MyNodeHandle node;
  ros::Subscriber depthRawSub = node.subscribe("/depth/image_raw", 1, depthCallback);
  ros::Subscriber commandSub = node.subscribe("/cmd_center/author", 1, commandCallback);
  cmdVelPub = node.advertise<geometry_msgs::Twist>("/goal_vel", 1);

  // get params
  ROS_INFO("obstacle_avoidance get params:");
  paused = node.getParamEx("avoidance/paused", false);
  maxLinear = node.getParamEx("avoidance/maxLinear", 0.3);
  maxAngular = node.getParamEx("avoidance/maxAngular", 1.5);
  goalRotationDepth = node.getParamEx("avoidance/goalRotationDepth", 450);
  goalFrontDepth = node.getParamEx("avoidance/goalFrontDepth", 450);
  minRotationPoints = node.getParamEx("avoidance/minRotationPoints", 50);
  maxRotationDepth = node.getParamEx("avoidance/maxRotationDepth", 800);
  minFrontPoints = node.getParamEx("avoidance/minFrontPoints", 300);
  maxFrontDepth = node.getParamEx("avoidance/maxFrontDepth", 1000);
  frontWidth = node.getParamEx("avoidance/frontWidth", 300);
  frontHeight = node.getParamEx("avoidance/frontHeight", 40);
  rotationWidth = node.getParamEx("avoidance/rotationWidth", 100);
  rotationHeight = node.getParamEx("avoidance/rotationHeight", 60);
  isSonarOn = node.getParamEx("avoidance/isSonarOn", false);
  minSonarFront = node.getParamEx("avoidance/minSonarFront", 0.40);

  if (isSonarOn)
  {
    ros::Subscriber frontSub = node.subscribe("/sonar/front", 1, frontCallback);
    ros::Subscriber leftSub = node.subscribe("/sonar/left", 1, leftCallback);
    ros::Subscriber rightSub = node.subscribe("/sonar/right", 1, rightCallback);
  }

  ros::spin();

  return 0;
}
