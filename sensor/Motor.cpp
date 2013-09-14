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
  initMotor(pinPwm, pinA, pinB, freq);
}

Motor::~Motor()
{
  // TODO Auto-generated destructor stub
}

void Motor::initMotor(int pinPwm, int pinA, int pinB, int freq)
{
  this->pinPwm = pinPwm;
  this->pinA = pinA;
  this->pinB = pinB;
  init();
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
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
    int value = step * duty;
    analogWrite(pinPwm, value);
  }
  else
  {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    int value = -step * duty;
    analogWrite(pinPwm, value);
  }
}

void Motor::test(int pinPwm, int pinA, int pinB, int freq, float duty)
{
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  printf("Turn [%d] [%d]\n", pinA, pinB);
 /* int step = pwmfreq_set(pinPwm, freq);
  printf("PWM%d set freq %d and valid duty cycle range [0, %d]\n", pinPwm, freq, step);
  if (step > 0)
  {
    int value = duty * step;
    printf("PWM%d test with duty cycle %d\n", pinPwm, value);
    analogWrite(pinPwm, value);
  }*/
  for (int i = 0; i < 100; i++)
  {
  digitalWrite(pinA, LOW);
	  digitalWrite(pinB, HIGH);
    int v6 = digitalRead(6);
	int v7 = digitalRead(7);
	  printf("v6=[%d] v7=[%d]\n", v6, v7);
	  delay(100); 
  }
}
