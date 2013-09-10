#include <ros/ros.h>
#include <Arduino.h>
#include "wiring_private.h"
#include "SR04.h"
#include <vector>

int main(int argc, char **argv)
{
	// init ros
	ros::init(argc, argv, "test_sensor3");
	ros::NodeHandle node;
	init();
	int echoPins[3] = {8, 10, 12};
	int trigPins[3] = {9, 11, 13};
	std::vector<SR04> sonars;
	for (int i = 0; i < 3; i++)
	{
		SR04 sr04(echoPins[i], trigPins[i]);
		sonars.push_back(sr04);
	}

	ros::Rate loopRate(4);
	while (ros::ok())
	{
		ROS_INFO("Front-left-right(cm): [%lu]-[%lu]-[%lu]", sonars[0].Distance(), sonars[1].Distance(), sonars[2].Distance());
        ROS_INFO("Avg: [%lu]-[%lu]-[%lu]", sonars[0].DistanceAvg(0, 5), sonars[1].DistanceAvg(0, 5), sonars[2].DistanceAvg(0, 5));
		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

