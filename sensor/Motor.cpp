/*
 * Motor.cpp
 *
 *  Created on: Sep 12, 2013
 *      Author: viki
 */

#include "Motor.h"
#include <Arduino.h>
#include <wiring_private.h>

Motor::Motor()
{
}

Motor::Motor(int pinPwm, int pinA, int pinB, int freq)
{
  init(pinPwm, pinA, pinB, freq);
}

Motor::~Motor()
{
  // TODO Auto-generated destructor stub
}

void Motor::init(int pinPwm, int pinA, int pinB, int freq)
{
  this->pinPwm = pinPwm;
  this->pinA = pinA;
  this->pinB = pinB;
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  int step = 0;
  step = pwmfreq_set(pinPwm, freq);
  printf("PWM%d set freq %d and valid duty cycle range [0, %d]\n", pinPwm, freq, step);
}

void Motor::run(float duty)
{
  if (fabs(duty) > 1)
  {
    printf("Please set duty between -1~1");
    duty = duty > 0 ? 1 : -1;
  }
  if (duty > 0)
  {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    int value = MAX_PWM_LEVEL * duty;
    analogWrite(pinPwm, value);
  }
  else
  {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    int value = -MAX_PWM_LEVEL * duty;
    analogWrite(pinPwm, value);
  }
}
