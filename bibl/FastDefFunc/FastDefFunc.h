#pragma once
#include <Arduino.h>


void pinModeFast(uint8_t pin, uint8_t mode);
void digitalWriteFast(uint8_t pin, bool x);
void analogWriteFast(uint8_t pin, uint16_t duty);
bool digitalReadFast(uint8_t pin);
uint16_t analogReadFast(uint8_t pin);

