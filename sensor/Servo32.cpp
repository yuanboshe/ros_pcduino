/*
 * Servo32.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: Yuanbo She
 */

#include "Servo32.h"
#include <Arduino.h>
#include <wiring_private.h>
#include "Serial.h"

Servo32::Servo32()
{
  // empty
}

Servo32::Servo32(int baud)
{
  initServo32(baud);
}

Servo32::~Servo32()
{
  // TODO Auto-generated destructor stub
}

void Servo32::initServo32(int baud)
{
  Serial.begin(baud);
  Serial.println("Servo32 started...");
}

/**
 * position: 500->-90 900->-45 1500->0 2100->45 2500->90
 * velocity: us/s numbers of us per second
 */
void Servo32::run(std::vector<int> pin, std::vector<int> position, std::vector<int> velocity)
{
  int size = pin.size();
  if (size > velocity.size())
  {
    for (int i = 0; i < size; i++)
    {
      Serial.print("#");
      Serial.print(pin[i]);
      Serial.print("P");
      Serial.print(position[i]);
    }
  }
  else
  {
    for (int i = 0; i < size; i++)
    {
      Serial.print("#");
      Serial.print(pin[i]);
      Serial.print("P");
      Serial.print(position[i]);
      Serial.print("S");
      Serial.print(velocity[i]);
    }
  }
  Serial.println("");
}

void Servo32::test(std::vector<int> pin, std::vector<int> position, std::vector<int> velocity)
{
  for (int i = 0; i < pin.size(); i++)
  {
    printf("%d %f ", pin[i], position[i]);
  }
  printf("\n");
}
