/*
 * Servo32.h
 *
 *  Created on: Sep 23, 2013
 *      Author: Yuanbo She
 */

#ifndef SERVO32_H_
#define SERVO32_H_

#include <vector>

class Servo32
{
public:
  Servo32();
  Servo32(int baud);
  virtual ~Servo32();
  void initServo32(int baud);
  void run(std::vector<int> pin, std::vector<int> position, std::vector<int> velocity);
  void test(std::vector<int> pin, std::vector<int> position, std::vector<int> velocity);
};

#endif /* SERVO32_H_ */
