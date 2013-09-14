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
  Motor(int pinPwm, int pinA, int pinB, int freq = 781);
  virtual ~Motor();
  int pinPwm;
  int pinA;
  int pinB;
  int step;
  void initMotor(int pinPwm, int pinA, int pinB, int freq = 781);
  void run(float duty);
  void test(int pinPwm, int pinA, int pinB, int freq, float duty);
};

#endif /* MOTOR_H_ */
