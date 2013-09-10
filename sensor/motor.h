#ifndef ___H_MOTO
#define ___H_MOTO
#include <Arduino.h>
//void moto_pin_init(char pin1, char pin2);
//
//void moto_pwm_set(int pwm_id, int freq, int value);
//
//void set_moto_turn_to(int pin1, int pin2, int turn1, int turn2);

void moto_pin_init(char pin1, char pin2)
{
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
}
/*
    pwm_id: the pin for pwm
    freq:Frequency
    value:duty cycle
*/
void moto_pwm_set(int pwm_id, int freq, int value)
{
    int step = 0;
    //printf("Usage PIN_ID(3/9/10/11) Frequency[125-2000]Hz Duty Level     or PIN_ID(5/6) Frequency[195,260,390,520,781]Hz Duty Level\n ");
    step = pwmfreq_set(pwm_id,freq);
    if(step > 0)
    {
        //printf("PWM%d test with duty cycle %d\n", pwm_id, value);
        analogWrite(pwm_id, value);
    }
    else
    {
        return;
    }
}
/*
    example:
     set_moto_turn_to(1,2,HIGH,LOW);
*/
void set_moto_turn_to(int pin1, int pin2, int turn1, int turn2)
{
    digitalWrite(pin1, turn1);
    digitalWrite(pin2, turn2);
}

#endif
