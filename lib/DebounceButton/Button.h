#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <Arduino.h>

#define NORMAL_STATE HIGH
#define PRESSED_STATE LOW

void buttonInit(const uint8_t *pins, uint8_t numButtons);
void getKeyInput();
bool isButtonPressed(uint8_t index);

#endif
