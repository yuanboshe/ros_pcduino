#ifndef ___H_MOTO
#define ___H_MOTO
void moto_pin_init(char pin1, char pin2);

void moto_pwm_set(int pwm_id, int freq, int value);

void set_moto_turn_to(int pin1, intpin2, int turn1, int turn2);
