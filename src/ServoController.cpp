/*
 * ServoController.cpp
 *
 *  Created on: Sep 24, 2013
 *      Author: viki
 */

#include "ServoController.h"

ServoController::ServoController()
{
  // TODO Auto-generated constructor stub

}

ServoController::~ServoController()
{
  // TODO Auto-generated destructor stub
}

ServoController::ServoController(int baud, std::vector<std::string> names, std::vector<int> pins, double radRate)
{
  initServoController(baud, names, pins, radRate);
}

void ServoController::initServoController(int baud, std::vector<std::string> names, std::vector<int> pins, double radRate)
{
  servo32.initServo32(baud);
  this->middlePosition = 1500;
  this->radRate = radRate;
  int size = names.size();
  for (int i = 0; i < size; i++)
  {
    nameMap.insert(std::pair<std::string, int>(names[i], pins[i]));
    posMap.insert(std::pair<std::string, int>(names[i], 0));
  }
}

void ServoController::run(std::vector<std::string> name, std::vector<double> position, std::vector<double> velocity)
{
  int size = name.size();
  bool existVel = (size == velocity.size());
  bool posIsDiff = false;
  std::vector<int> pin;
  std::vector<int> rvPosition;
  std::vector<int> rvVelocity;
  for (int i = 0; i < size; i++)
  {
    std::map<std::string, int>::iterator iter = nameMap.find(name[i]);
    if (iter != nameMap.end())
    {
      std::map<std::string, int>::iterator posIter = posMap.find(name[i]);
      int pinId = iter->second;
      int newPos = position[i] * radRate + 1500;
      int& oldPos = posIter->second;
      if (newPos != oldPos)
      {
        oldPos = newPos;
        posIsDiff = true;
      }
      pin.push_back(pinId);
      rvPosition.push_back(newPos);
      if (existVel) rvVelocity.push_back(velocity[i] * radRate);
    }
  }
  if (posIsDiff) servo32.run(pin, rvPosition, rvVelocity);
}
