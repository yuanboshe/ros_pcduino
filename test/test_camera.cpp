#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "MyNodeHandle.h"
#include "Utility.h"

using namespace cv;

// params
int paramMinRange, paramMaxRange;

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

  // init2
  float range[2] = {paramMinRange, paramMaxRange};
  float maxWeight = 255;
  int depth = 0;
  float weight = 0;

  // calc weight mat
  Mat_<float> wMat;
  Point3f center;
  center = util.calcWeightCneter(depthImg, wMat, range, 500, maxWeight);

  Mat_<Vec3b> depthColorImg(rows, cols);
  for (int i = 0; i < rows; i++)
  {
    int16_t* p0 = depthImg.ptr<int16_t>(i);
    float* p1 = wMat.ptr<float>(i);
    Vec3b* p2 = depthColorImg.ptr<Vec3b>(i);
    for (int j = 0; j < cols; j++)
    {
      p2[j] = Vec3b(p1[j], p1[j], p1[j]);
      if (i == 200 && j == 200)
      {
        depth = p0[j];
        weight = p1[j];
        p2[j - 1] = p2[j - 2] = Vec3b(0, 0, 255);
      }
    }
  }

  // draw center point
  Point centerPos((int)center.x, (int)center.y);
  circle(depthColorImg, centerPos, 6, Scalar(0, 255, 0), -1);

  // put text
  Point pos(20, 40);
  Scalar textColor(0, 200, 0);
  string text("red point depth: ");
  text.append(boost::lexical_cast<string>(depth));
  putText(depthColorImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);
  pos.y += 20;
  text = "red point weight: ";
  text.append(boost::lexical_cast<string>(weight));
  putText(depthColorImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);
  pos.y += 20;
  text = "minDepth of all points: ";
  text.append(boost::lexical_cast<string>(util.getMinDepth(depthImg, 10)));
  putText(depthColorImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);
  pos.y += 20;
  text = "size: ";
  text.append(boost::lexical_cast<string>(cols));
  text.append(" * ");
  text.append(boost::lexical_cast<string>(rows));
  putText(depthColorImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);
  pos.y += 20;
  text = "avZ: ";
  text.append(boost::lexical_cast<string>(center.z));
  putText(depthColorImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);

  imshow("camera info", depthColorImg);
  waitKey(10);
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "test_camera");
  MyNodeHandle node;
  ros::Subscriber depthRawSub = node.subscribe("/depth/image_raw", 10, depthCallback);

  // get params
  ROS_INFO("test_camera get params:");
  paramMinRange = node.getParamEx("test_camera/minRange", 500); // for ASUS xtion, set bigger than 500
  paramMaxRange = node.getParamEx("test_camera/maxRange", 2000);
  ROS_INFO("test_camera get params finish.");


  ros::spin();

  return 0;
}
