/*
 * Motor.h
 *
 *  Created on: Sep 12, 2013
 *      Author: viki
 */

#ifndef MOTOR_H_
#define MOTOR_H_

class Motor
{
public:
  Motor();
  Motor(int pinPwm, int pinA, int pinB, int freq = 500);
  virtual ~Motor();
  int pinPwm;
  int pinA;
  int pinB;
  void init(int pinPwm, int pinA, int pinB, int freq = 500);
  void run(float duty);
};

#endif /* MOTOR_H_ */
