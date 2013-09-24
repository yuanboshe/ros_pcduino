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
  }
}

void ServoController::run(std::vector<std::string> name, std::vector<double> position, std::vector<double> velocity)
{
  int size = name.size();
  std::vector<int> pin(size);
  std::vector<int> rvPosition(size);
  std::vector<int> rvVelocity(velocity.size());
  for (int i = 0; i < size; i++)
  {
    pin[i] = nameMap.find(name[i])->second;
    rvPosition[i] = position[i] * radRate + 1500;
  }
  if (size == velocity.size())
  {
    for (int i = 0; i < size; i++)
    {
      rvVelocity[i] = velocity[i] * radRate;
    }
  }
  servo32.run(pin, rvPosition, rvVelocity);
}
