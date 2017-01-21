#include <aducm360.h>
#include "keypad.h"
#include "haladucm360.h"

static uint8_t keyCode;
static uint8_t keyPressTime[16];
static uint8_t keyPressState;
static uint8_t keyRptCntr;
static uint8_t keyLast;
static uint16_t longKeyMask = 0x0;
void keypadInit(void)
{
  uint8_t i;
  for(i=0;i<16;i++) keyPressTime[i] = 0;
  keyCode = 0x0;
  keyPressState = NO_KEY_PRESSED;
  keyRptCntr = 0;
}

uint8_t getPressedKey()
{
  return keyCode;
}

void keyProcessed()
{
  if(keyPressState == REPEAT_KEY_PRESSED){
    keyRptCntr++;
    if(keyRptCntr < REPEAT_KEY_DELAY){
      keyPressTime[keyCode] -= REPEAT_SPEED_LOW;
    }else{
      keyPressTime[keyCode] -= REPEAT_SPEED_HIGH;
    }
  }else{
    keyPressTime[keyCode] = 0;
    keyRptCntr = 0;
  }
}

int16_t keypadPoll(uint32_t ct)
{
  int16_t keyPressed = 0;
  uint16_t keyState = halReadButtons();
  keyState = (~keyState) & 0xf;
  keyPressState = NO_KEY_PRESSED;
  if(keyState != 0x0){
    keyPressTime[keyState]++;
    keyCode = keyState;
    if((keyState == keyUp) || (keyState == keyDn)){
      if(keyPressTime[keyState] > REPEAT_COUNTER){
        //changeTime = ctime;
        keyPressState = REPEAT_KEY_PRESSED;
        keyPressed = 1;
        //keyCode = keyState;
      }
    }
    else{
      if(keyPressTime[keyCode] > PRESS_LONG_PERIOD){
        keyPressState = LONG_KEY_PRESSED;
        //keyCode = keyState;
        keyPressed = LONG_KEY_PRESSED;
//        keyPressTime[keyCode] = 0;
        longKeyMask |= (1 << keyCode);
      }
        
    }
  }else{
    if(longKeyMask != 0x0){
      keyPressTime[keyCode] = 0;      
      longKeyMask &= ~(1 << keyCode);
    }
    if(keyPressTime[keyCode] > PRESS_SHORT_PERIOD){
      keyPressState = SHORT_KEY_PRESSED;
      //keyCode = keyState;
      keyPressed = SHORT_KEY_PRESSED;
      keyPressTime[keyCode] = 0;
      }
    
  }
  //keyLast = keyState;
  return keyPressed;
}