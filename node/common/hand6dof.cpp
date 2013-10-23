#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/JointState.h>
#include "ServoController.h"

using namespace cv;

ServoController servoController;

std::vector<std::string> name(6);
std::vector<int> pin(6);
void depthCenterCallback(const geometry_msgs::Point32ConstPtr& msg)
{
  try
  {
    Point3f point;
    point.x = msg->x;
    point.y = msg->y + 0.4;
    point.z = msg->z - 40;
    // control
    if (msg->z > 0)
    {
      std::vector<double> pos(6);
      pos[0] = -point.x;
      pos[1] = -point.y * 0.3;
      pos[3] = -point.y * 0.5;
      pos[2] = point.y * 0.6;
      pos[4] = 0;
      pos[5] = -point.z / 250;
      std::vector<double> vel(6);
      for (int i = 0; i < 6; i++)
      {
        vel[i] = 0.8;
      }
      ROS_INFO("depth run: %d %d", name.size(), pos.size());
      servoController.run(name, pos, vel);
    }
    ROS_INFO("skip z=[%f]", msg->z);
  }
  catch (Exception& e)
  {
    ROS_ERROR("follower depthCallback exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "hand6dof");
  ros::NodeHandle node;
  ros::Subscriber depthCenterSub = node.subscribe("/depth_center", 1, depthCenterCallback);

  // init ServoController
  name[0] = "pan";
  name[1] = "bend1";
  name[2] = "bend2";
  name[3] = "bend3";
  name[4] = "rotate";
  name[5] = "clip";
  pin[0] = 0;
  pin[1] = 1;
  pin[2] = 2;
  pin[3] = 3;
  pin[4] = 4;
  pin[5] = 5;
  servoController.initServoController(115200, name, pin);

  ROS_INFO("hand6dof started...");
  ros::spin();

  return 0;
}
