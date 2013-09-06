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

VideoWriter histWriter;
VideoWriter depthWriter;
VideoWriter rgbWriter;
Size histSize(320, 240);
Size depthSize(640, 480);
Size rgbSize(640, 480);

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
    // weight
    int maxRange = maxZ;
    Mat_<float> wMat(rows, cols); // weight mat
    wMat.setTo(0);
    float wSum = 0;
    double xSum, ySum, zSum;
    xSum = ySum = zSum = 0;
    int pointsSum = 0; // points sum which in the range
    for (int i = 0; i < rows; i++)
    {
      uint16_t* pDepthRow = depthImg.ptr<uint16_t>(i);
      float* pwMatRow = wMat.ptr<float>(i);
      for (int j = weightMarggin; j < cols - weightMarggin; j++)
      {
        int z = pDepthRow[j]; // depth of current point
        double w = 0; // weight of current point
        if (z < maxRange) // if z in the range, calc the w
        {
          w = (1 - (double)pDepthRow[j] / maxRange) * maxWeight;
          pwMatRow[j] = w;
          wSum += w;
          xSum += j * w;
          ySum += i * w;
          zSum += z * w;
          pointsSum++;
        }
      }
    }

    // control
    float centerX = cols / 2;
    float avX, avY, avZ;
    if (pointsSum > minPoints)
    {
      avX = xSum / wSum;
      avY = ySum / wSum;
      avZ = zSum / wSum;
      float linearBias = avZ - goalZ;
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

    // draw depth color image
    bool channals[3] = {true, false, false};
    Mat depthColorImg = util.depth2Color(depthImg, 2000, channals);

    // draw depth color image advance
    float depthRange[2] = {0, 2000};
    bool discard[2] = {false, true};
    bool depthChannals[3] = {false, false, true};
    Rect roiRect = Rect(weightMarggin, 0, depthColorImg.cols - 2 * weightMarggin, depthColorImg.rows);
    Mat depthRoi = depthImg(roiRect);
    Mat depthColorRoi = depthColorImg(roiRect);
    util.depth2Color(depthRoi, depthColorRoi, depthRange, discard, depthChannals);

    Point center((int)avX, (int)avY);
    circle(depthColorImg, center, 6, Scalar(0, 255, 0), -1);

    // put text info
    resize(depthColorImg, depthColorImg, depthSize);
    Point pos(20, 40);
    Scalar textColor(0, 200, 0);
    string text("time: ");
    text.append(boost::lexical_cast<string>(ros::Time::now()));
    putText(depthColorImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);
    pos.y += 20;
    text = "linear: ";
    text.append(boost::lexical_cast<string>(linearSpeed));
    putText(depthColorImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);
    pos.y += 20;
    text = "angular: ";
    text.append(boost::lexical_cast<string>(angularSpeed));
    putText(depthColorImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);

    imshow("center", depthColorImg);
    waitKey(10);

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
