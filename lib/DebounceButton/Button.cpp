#include "button.h"

static const uint8_t *buttonPins = NULL; // Pointer to the button pins array
static uint8_t NUM_BUTTONS = 0;

int *keyReg0 = NULL;
int *keyReg1 = NULL;
int *keyReg2 = NULL;
int *keyRegStable = NULL;
bool *button_flag = NULL;

void buttonInit(const uint8_t *pins, uint8_t numButtons) {
    buttonPins = pins;
    NUM_BUTTONS = numButtons;

    keyReg0 = new int[NUM_BUTTONS];
    keyReg1 = new int[NUM_BUTTONS];
    keyReg2 = new int[NUM_BUTTONS];
    keyRegStable = new int[NUM_BUTTONS];
    button_flag = new bool[NUM_BUTTONS];

    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
        keyReg0[i] = NORMAL_STATE;
        keyReg1[i] = NORMAL_STATE;
        keyReg2[i] = NORMAL_STATE;
        keyRegStable[i] = NORMAL_STATE;
        button_flag[i] = false;
    }
}

void getKeyInput() {
    if (!buttonPins) return;

    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        keyReg2[i] = keyReg1[i];
        keyReg1[i] = keyReg0[i];
        keyReg0[i] = digitalRead(buttonPins[i]);

        if ((keyReg0[i] == keyReg1[i]) && (keyReg1[i] == keyReg2[i])) {
            if (keyRegStable[i] != keyReg0[i]) {
                keyRegStable[i] = keyReg0[i];
                if (keyRegStable[i] == PRESSED_STATE) {
                    button_flag[i] = true;
                }
            }
        }
    }
}

bool isButtonPressed(uint8_t index) {
    if (index >= NUM_BUTTONS) return false;

    if (button_flag[index]) {
        button_flag[index] = false;
        return true;
    }
    return false;
}
