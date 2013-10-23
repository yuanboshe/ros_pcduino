#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "Utility.h"

using namespace cv;

ros::Publisher depthCenterPub;
void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
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
    float range[2] = {10, 300};
    Mat_<float> wMat;
    Point3f center;
    center = util.calcWeightCneter2(depthImg, wMat, range, 5000, 3.0, 0);

    geometry_msgs::Point32 pointPub;
    pointPub.x = center.x;
    pointPub.y = center.y;
    pointPub.z = center.z;
    depthCenterPub.publish(pointPub);
  }
  catch (Exception& e)
  {
    ROS_ERROR("depth_center depthCallback exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "depth_center");
  ros::NodeHandle node;
  ros::Subscriber depthRawSub = node.subscribe("/depth/image_raw", 1, depthCallback);
  depthCenterPub = node.advertise<geometry_msgs::Point32>("/depth_center", 1);

  ROS_INFO("depth_center started...");
  ros::spin();

  return 0;
}
