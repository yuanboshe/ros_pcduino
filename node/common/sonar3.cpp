#include <ros/ros.h>
#include <Arduino.h>
#include "wiring_private.h"
#include "SR04.h"
#include <list>
#include <sensor_msgs/Range.h>
#include "MyNodeHandle.h"

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "sonar3");
  MyNodeHandle node;
  ros::Publisher rangePubs[3];
  rangePubs[0] = node.advertise<sensor_msgs::Range>("/sonar/front", 100);
  rangePubs[1] = node.advertise<sensor_msgs::Range>("/sonar/left", 100);
  rangePubs[2] = node.advertise<sensor_msgs::Range>("/sonar/right", 100);

  // get params
  int rate = node.getParamEx("sonar3/rate", 6);
  int count = node.getParamEx("sonar3/count", 4);
  int frontEcho = node.getParamEx("sonar3/frontEcho", 8);
  int frontTrig = node.getParamEx("sonar3/frontTrig", 9);
  int leftEcho = node.getParamEx("sonar3/leftEcho", 10);
  int leftTrig = node.getParamEx("sonar3/leftTrig", 11);
  int rightEcho = node.getParamEx("sonar3/rightEcho", 12);
  int rightTrig = node.getParamEx("sonar3/rightTrig", 13);

  // init SR04 front-left-right
  init();
  int echoPins[3] = {frontEcho, leftEcho, rightEcho};
  int trigPins[3] = {frontTrig, leftTrig, rightTrig};
  std::list<int> bufs[3];
  std::vector<SR04> sonars;
  for (int i = 0; i < 3; i++)
  {
    SR04 sr04(echoPins[i], trigPins[i]);
    sonars.push_back(sr04);
    for (int j = 0; j < count + 2; j++)
    {
      bufs[i].push_back(0);
    }
  }

  // ros loop
  ros::Rate loopRate(rate);
  while (ros::ok())
  {
    // calc avg dists
    float avgs[3];
    for (int i = 0; i < 3; i++)
    {
      long min = 999, max = 0;
      float avg = 0;
      std::list<int>& buf = bufs[i];
      buf.pop_front();
      buf.push_back(sonars[i].Distance());
      for (std::list<int>::iterator it = buf.begin(); it != buf.end(); it++)
      {
        int d = *it;
        if (d < min)
          min = d;
        if (d > max)
          max = d;
        avg += d;
      }
      avg -= (max + min);
      avgs[i] = avg / count;

      // pub message
      sensor_msgs::Range sonarRange;
      sonarRange.range = avgs[i] / 100;
      rangePubs[i].publish(sonarRange);
    }

    ROS_INFO("Front-left-right(cm): [%f] - [%f] - [%f]", avgs[0], avgs[1], avgs[2]);
    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}

