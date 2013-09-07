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
  ros::Publisher rangePub = node.advertise<sensor_msgs::Range>("/sonar3", 100);

  // get params
  int count = node.getParamEx("sonar3/count", 5);

  // init SR04 front-left-right
  init();
  int echoPins[3] = {8, 10, 12};
  int trigPins[3] = {9, 11, 13};
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
  ros::Rate loopRate(10);
  while (ros::ok())
  {
    // calc avg dists
    float avgs[3];
    for (int i = 0; i < 3; i++)
    {
      long min = 0, max = 999;
      float avg = 0;
      std::list<int> buf = bufs[i];
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
    }

    ROS_INFO("Front-left-right(cm): [%f] - [%f] - [%f]", avgs[0], avgs[1], avgs[2]);
    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}

