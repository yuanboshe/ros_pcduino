#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "MyNodeHandle.h"
#include "Utility.h"
#include "ServoController.h"

using namespace cv;

// params
bool paused; // dynamic pause or resume this program

ServoController servoController;
void jointCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  servoController.run(msg->name, msg->position, msg->velocity);
}

std::vector<std::string> name(2);
std::vector<int> pin(2);
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
    float range[2] = {10, 300};
    Mat_<float> wMat;
    Point3f center;
    center = util.calcWeightCneter(depthImg, wMat, range, 5000, 3.0, 0);

    // control
    float yBias = rows / 2 - center.y;
    float xBias = cols / 2 - center.x;

    float yPos = 0;
    float xPos = 0;
    if (center.z > 0)
    {
      yPos = yBias * 2 / rows * 1;
      xPos = xBias * 2 / cols * 1;
      std::vector<double> pos(2);
      pos[0] = xPos;
      pos[1] = -yPos;
      std::vector<double> vel(2);
      vel[0] = 0.8;
      vel[1] = 0.8;
      ROS_INFO("depth run: %d %d", name.size(), pos.size());
      servoController.run(name, pos, vel);
    }
    ROS_INFO("skip z=[%f]", center.z);
  }
  catch (Exception& e)
  {
    ROS_ERROR("follower depthCallback exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "pantilt");
  MyNodeHandle node;
  ros::Subscriber jointStateSub = node.subscribe("/joint_states", 1, jointCallback);
  ros::Subscriber depthRawSub = node.subscribe("/depth/image_raw", 1, depthCallback);
  //ros::Subscriber commandSub = node.subscribe("/cmd_center/author", 1, commandCallback);

  paused = false;

  // init ServoController
  name[0] = "head_pan_joint";
  name[1] = "head_tilt_joint";
  pin[0] = 0;
  pin[1] = 1;
  servoController.initServoController(115200, name, pin);

  ROS_INFO("pantilt started...");
  ros::spin();

  return 0;
}
