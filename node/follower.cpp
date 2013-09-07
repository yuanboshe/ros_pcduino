#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "MyNodeHandle.h"
#include "Utility.h"

using namespace cv;

ros::Publisher cmdVelPub;
bool paused; // dynamic pause or resume this program

// params
int paramMinRange, paramMaxRange;
float paramMaxWeight;
int paramGoalZ;


int goalZ, maxZ, minPoints;
double maxLinear;
double maxAngular;
double linearRespRate, angularRespRate;
double maxWeight;
int weightMarggin;
bool isViewVideo;
bool isSaveVideo;
double linearSpeed = 0;
double angularSpeed = 0;

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
    float maxWeight = paramMaxWeight;
    Mat_<float> wMat;
    Point3f center;
    center = util.calcWeightCneter(depthImg, wMat, range, 500, maxWeight);

    // control
/*
    float linearBias = avZ - paramX;
    float angularBias = centerX - avX;

    linearSpeed = linearBias > 0 ? linearBias / (maxRange - goalZ) * maxLinear : linearBias / goalZ * maxLinear;
    angularSpeed = angularBias * 2 / cols * maxAngular;

    linearSpeed *= linearRespRate;
    angularSpeed *= angularRespRate;
    }

    // pub message
    geometry_msgs::Twist moveCmd;
    moveCmd.linear.x = linearSpeed;
    moveCmd.angular.z = angularSpeed;
    cmdVelPub.publish(moveCmd);
*/



  }
  catch (Exception& e)
  {
    ROS_ERROR("follower depthCallback exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "follower3");
  MyNodeHandle node;
  ros::Subscriber depthRawSub = node.subscribe("/depth/image_raw", 10, depthCallback);
  ros::Subscriber commandSub = node.subscribe("/cmd_center/author", 100, commandCallback);
  cmdVelPub = node.advertise<geometry_msgs::Twist>("/goal_vel", 100);

  // get params
  ROS_INFO("follower3 get params:");
  paused = node.getParamEx("follower3/paused", true);
  maxLinear = node.getParamEx("follower3/maxLinear", 0.5);
  maxAngular = node.getParamEx("follower3/maxAngular", 2.0);
  goalZ = node.getParamEx("follower3/goalZ", 600);
  maxZ = node.getParamEx("follower3/maxZ", 900);
  maxWeight = node.getParamEx("follower3/maxWeight", 8.0);
  weightMarggin = node.getParamEx("follower3/weightMarggin", 50);
  minPoints = node.getParamEx("follower3/minPoints", 1000);
  linearRespRate = node.getParamEx("follower3/linearRespRate", 1.0);
  angularRespRate = node.getParamEx("follower3/angularRespRate", 1.0);
  isViewVideo = node.getParamEx("follower3/isViewVideo", true);
  isSaveVideo = node.getParamEx("follower3/isSaveVideo", true);
  string histVideoPath = node.getParamEx("follower3/histVideoPath", "histogram.avi");
  string depthVideoPath = node.getParamEx("follower3/depthVideoPath", "follower.avi");
  string rgbVideoPath = node.getParamEx("follower3/rgbVideoPath", "rgb.avi");

  ros::spin();

  return 0;
}
