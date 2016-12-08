#ifndef _KEYPAD_
#define _KEYPAD_

#define NO_KEY_PRESSED          0x0
#define SHORT_KEY_PRESSED       0x01
#define LONG_KEY_PRESSED        0x02
#define REPEAT_KEY_PRESSED      0x04

#define PRESS_LONG_PERIOD		30		// counts
#define PRESS_SHORT_PERIOD		2		// counts
#define REPEAT_COUNTER  		8
#define REPEAT_SPEED_LOW               4
#define REPEAT_SPEED_HIGH               2
#define REPEAT_KEY_DELAY                10

enum keyCode_e{
  keyCycle=0x01,
  keyUp=0x02,
  keyAuto = 0x03,
  keyDn=0x04,
  keyMod=0x05,
  keyRet = 0x06,
  keyRm=0x08,
  keyProIn=0xa,
  keyProOut=0xc
};

void keypadInit(void);
uint8_t getPressedKey();
int16_t keypadPoll(uint32_t ct);
void keyProcessed();
#endif