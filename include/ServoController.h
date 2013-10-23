/*
 * ServoController.h
 *
 *  Created on: Sep 24, 2013
 *      Author: viki
 */

#ifndef SERVOCONTROLLER_H_
#define SERVOCONTROLLER_H_

#include <string>
#include <map>
#include "Servo32.h"

class ServoController
{
public:
  ServoController();
  virtual ~ServoController();
  std::map<std::string, int> nameMap;
  std::map<std::string, int> posMap;
  Servo32 servo32;
  double radRate; // us / rad  e.g. 1200 / (pi / 2)
  int middlePosition; // us
  ServoController(int baud, std::vector<std::string> names, std::vector<int> pins, double radRate = 763.94);
  void initServoController(int baud, std::vector<std::string> names, std::vector<int> pins, double radRate = 763.94);
  void run(std::vector<std::string> name, std::vector<double> position, std::vector<double> velocity = std::vector<double>());
};

#endif /* SERVOCONTROLLER_H_ */
