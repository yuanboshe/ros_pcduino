#include <ros/ros.h>
#include <Arduino.h>
#include "wiring_private.h"
#include "SR04.h"
#include <queue>
#include <sensor_msgs/Range.h>
#include "MyNodeHandle.h"

int main(int argc, char **argv)
{
	// init ros
	ros::init(argc, argv, "sonar3");
	MyNodeHandle node;
	ros::Publisher rangePub = node.advertise<sensor_msgs::Range>("/sonar3", 100);

	// get params
	int smooth = node.getParamEx("sonar3/smooth", 5);

	// init SR04 front-left-right
    init();
	int echoPins[3] = {8, 10, 12};
	int trigPins[3] = {9, 11, 13};
    std::queue<int> bufs[3];
	std::vector<SR04> sonars;
	for (int i = 0; i < 3; i++)
	{
		SR04 sr04(echoPins[i], trigPins[i]);
		sonars.push_back(sr04);
		for (int j = 0; j < smooth; j++)
		{
		  bufs[i].push(0);
		}
	}

	// ros loop
	ros::Rate loopRate(4);
	while (ros::ok())
	{
	  for (int i = 0; i < 3; i++)
	  {
	    bufs[i].pop();
	    bufs[i].push(sonars[i].Distance());
	  }
		ROS_INFO("Front-left-right(cm): [%lu] - [%lu] - [%lu]", sonars[0].Distance(), sonars[2].Distance(), sonars[1].Distance());
		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

