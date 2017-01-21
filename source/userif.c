#include <aducm360.h>
#include <i2cLib.h>
#include <dmalib.h>
#include <diolib.h>
#include "config.h"
#include "userif.h"
#include "param.h"
#include "keypad.h"
#include "sysState.h"
#define I2C_SLA 0xe0

typedef struct{
  uint8_t reload;
  uint8_t cntr;
}_led_state_t;
/*
static void delay(uint32_t d)
{
  uint32_t j = d;
  while(j--);
}
*/

uint8_t charToSevenSeg[]={
  0x40,
  0x00,
  0x00,
  0x3f, //0
  0x06,
  0x5b,
  0x4f,
  0x66,
  0x6d,
  0x7d,
  0x07,
  0x7f,
  0x6f,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x77, // A.
  0x7c, // B.
  0x39,
  0x5e,
  0x79,
  0x71,
  0x3d, //G
  0x76,
  0x10,
  0x0e,
  0x7a,
  0x38, // L
  0x55,
  0x54,
  0x5c,
  0x73,//p
  0x67,
  0x50,
  0x6d,
  0x78,
  0x1c,
  0x6a, //v
  0x1d,
  0x36,
  0x3e,
  0x49
};

static _led_state_t ledState[4];
static uint8_t DispData[8];
static uint8_t dispChanged= 0;
static uint32_t idleTime = 0;

gpio_config_s leds[] = {
  {pADI_GP0,(1<<0),0},
  {pADI_GP0,(1<<1),0},
  {pADI_GP0,(1<<2),0},
  {pADI_GP0,(1<<3),0},  
};

void delay(long int length)
{
  while(length > 0) length--;
}

int16_t changeDisplayIndex(uint8_t index)
{
  
  return 0;
}

int16_t changeDisplayMode(uint8_t mode)
{
  
  return 0;
}

void uiSetDispContent(uint8_t *c)
{  
  uint8_t data[6]={0};
  uint8_t i;
  for(i=0;i<3;i++){
    DispData[i*2] = 0x0;
    if(c[i] >= 0x2d)
      DispData[i*2+1] = charToSevenSeg[c[i] - 0x2d];
    else
      DispData[i*2+1] = 0x0;
  }
  
  //delay(0xf0000);
  DmaClr(DMARMSKCLR_I2CMTX,0,0,0);
  DmaSet(0,DMAENSET_I2CMTX,0,DMAPRISET_I2CMTX);
  DmaStructPtrOutSetup(I2CMTX_C,6,DispData);
  DmaCycleCntCtrl(I2CMTX_C,6,DMA_DSTINC_NO | DMA_SRCINC_BYTE | DMA_SIZE_BYTE | DMA_BASIC);
  I2cMWrCfg(I2C_SLA);
  delay(0xffff);
}

void uiSetDispFlash(uint8_t f)
{
  DmaClr(DMARMSKCLR_I2CMTX,0,0,0);
  DmaSet(0,DMAENSET_I2CMTX,0,DMAPRISET_I2CMTX);
  if(f == 0)
    DispData[0] = 0x81;
  else
    DispData[0] = 0x83;
  DmaStructPtrOutSetup(I2CMTX_C,1,DispData);
  DmaCycleCntCtrl(I2CMTX_C,1,DMA_DSTINC_NO | DMA_SRCINC_BYTE | DMA_SIZE_BYTE | DMA_BASIC);
  I2cMWrCfg(I2C_SLA);
  delay(0xf0000);
   
}

void uiDispSetEnable(void)
{
  
  DmaClr(DMARMSKCLR_I2CMTX,0,0,0);
  DmaSet(0,DMAENSET_I2CMTX,0,DMAPRISET_I2CMTX);
  DispData[0] = 0x21;
  DmaStructPtrOutSetup(I2CMTX_C,1,DispData);
  DmaCycleCntCtrl(I2CMTX_C,1,DMA_DSTINC_NO | DMA_SRCINC_BYTE | DMA_SIZE_BYTE | DMA_BASIC);
  I2cMWrCfg(I2C_SLA);
  delay(0xf0000);

  uiSetDispFlash(0);
  
}

void uiDispSetBrightness(uint8_t b)
{
  if(b > 0xf) return;
  uint8_t bright = 0xe0 | b;
  DmaClr(DMARMSKCLR_I2CMTX,0,0,0);
  DmaSet(0,DMAENSET_I2CMTX,0,DMAPRISET_I2CMTX);
  DmaStructPtrOutSetup(I2CMTX_C,1,&bright);
  DmaCycleCntCtrl(I2CMTX_C,1,DMA_DSTINC_NO | DMA_SRCINC_BYTE | DMA_SIZE_BYTE | DMA_BASIC);
  I2cMWrCfg(I2C_SLA);
}

void uiLedSet(uint8_t id, uint8_t freq)
{
  if(id > 4) return;
  switch(freq){
  case 0:
    ledState[id].reload = LED_EN_MASK;
    leds[id].port->GPSET = ( leds[id].pin);
    break;
  case 1:
    ledState[id].reload = 0xa ;
    ledState[id].reload |= LED_EN_MASK;
    break;
  case 2:
    ledState[id].reload = 0x5 ;
    ledState[id].reload |= LED_EN_MASK;
    break;
  case 3:
    ledState[id].reload = 0x3 ;
    ledState[id].reload |= LED_EN_MASK;
    break;
  default:
    ledState[id].reload = 0x1 ;
    ledState[id].reload |= LED_EN_MASK;
    break;
  }
}

void uiLedClear(uint8_t id)
{
  if(id > 4) return;
  // todo : set led off
  ledState[id].reload = 0x0;
}

void uiInit(void)
{
  uiDispSetEnable();
  uiSetDispContent("000");
  uint8_t i;
  for(i=0;i<4;i++){
    ledState[i].cntr = ledState[i].reload = 0;
  }
  /*
  char str[3];
  getDisplayString(str);
  uiSetDispContent(str);
  */
}

int16_t uiLedPoll(uint32_t ct)
{
  int8_t reload, ledEn;
  uint8_t i;
  for(i=0;i<4;i++){
    reload = ledState[i].reload & RELOAD_MASK;
    ledEn = ledState[i].reload & LED_EN_MASK;
    if(ledEn){
      if(reload > 0){
        ledState[i].cntr++;
        if(reload < ledState[i].cntr){
          ledState[i].cntr = 0;
          // todo : toggle led
          leds[i].port->GPTGL = leds[i].pin;
        }
      }else{
        leds[i].port->GPSET = leds[i].pin;
      }
    }else{
      leds[i].port->GPCLR = leds[i].pin;
    }
  }
  // check for keys
    // update display
    char disp[3];
    paramGetDisplayString(disp);
    uiSetDispContent(disp);

  return 0;
}

int16_t uiKeyPoll(uint32_t ct)
{
  int16_t keyState;
  int16_t kv;
 
  if((keyState = keypadPoll(ct)) != NO_KEY_PRESSED){
    uint8_t keyCode = getPressedKey();
    switch(keyState){
    case SHORT_KEY_PRESSED:
      switch(keyCode){
      case keyCycle:
        paramMoveAdv();
        break;
      case keyUp:
        paramValueInc();
        break;
      case keyDn:
        paramValueDec();
        break;
      case keyMod:
        paramSetActiveOrder(ODR_CFG);        
        break;
      case keyRm:
        switch(paramGetCurrentOrder()){
        case ODR_IDLE:
          switch(sysGetOpMode()){
          case modPRO:
            // should be pause
            sysSetProPause();
            break;
          default:
            sysResetError();
          }
          break;
        case ODR_MAN:
          paramSetActiveOrder(ODR_IDLE);
          sysSetOP1Output(0);
          sysReturnOpMode();
          break;
        case ODR_ATO:
          paramSetActiveOrder(ODR_IDLE);
          sysReturnOpMode();
          break;
        default:
          if(paramGetShowTarget() == SHOW_STRING){
            paramSetActiveOrder(ODR_IDLE);
            sysNotifyParamAltered();
          }else{
            paramSetShowTarget(SHOW_STRING);
          }
        }
        break;
      case keyProIn:
        readIntParamByIndex(MOD,&kv);
        switch(kv){
        case modPRO:
          //sysSetOpMode(modPRO);
          sysSetProStart(0);
          break;
        }
        break;
      case keyProOut:
        switch(sysGetOpMode()){
        case modPRO:
          sysSetProStop();
          break;
        default:
          sysReturnOpMode();
        }
        break;
      case keyAuto:
        paramSetActiveOrder(ODR_ATO);
        sysSetOpMode(modATO);
        break;
      }
      break;
    case LONG_KEY_PRESSED:
      switch(keyCode){
      case keyCycle:
        paramSetActiveOrder(ODR_SET);        
        break;
      case keyRm:
        if(paramGetCurrentOrder() == ODR_IDLE){
          paramSetActiveOrder(ODR_MAN);        
          //sysSetOpMode(modMAN);
          //sysSetManOP1(0);
        }        
        break;
      }
      break;
    case REPEAT_KEY_PRESSED:
      switch(keyCode){
      case keyUp:
        paramValueInc();
        break;
      case keyDn:
        paramValueDec();
        break;
      }
      break;
    }
    keyProcessed();
    
  }

  // update leds
  int16_t mode = 0;
  uint8_t mv;
  mv = sysGetOP1Output();
  if(sysOpmodeAltered()){
    idleTime = ct;
    //resetSysInfo(MODE_ALTERED);
    mode = sysGetOpMode();
    switch(mode){
    case modATO:
      uiLedSet(MAN_LED,0);
      uiSetDispFlash(1);
      break;
    case modMAN:
      uiLedSet(MAN_LED,0);
      break;
    case modPRO:
      //sysSetProStart();
      uiLedSet(PRO_LED,0);
      break;
    default:
      uiLedClear(MAN_LED);
      uiSetDispFlash(0);
      mv = sysGetOP1Output();
    }
  }else{
    if((ct - idleTime) > 4000){
      uint8_t codr = paramGetCurrentOrder();
      //if((codr == ODR_SET) || (codr == ODR_CFG))
        //paramSetActiveOrder(ODR_IDLE);
    }
  }
  // mv led control by mv1
  if(mv == 0){
    uiLedClear(MV_LED);
  }else if(mv < 100){
    uiLedSet(MV_LED, mv/10);
  }else{
    uiLedSet(MV_LED,0);
  }
  
  mv = sysGetOP2Output();
  if(mv == 1){
    uiLedSet(MV2_LED,0);
  }else{
    uiLedClear(MV2_LED);
  }
  
  if(sysGetOpMode() == modPRO){
    switch(sysGetProState()){
    case PROS_IDLE:
      uiLedClear(PRO_LED);
      break;
    case PROS_RUN:
      uiLedSet(PRO_LED,0);
      break;
    case PROS_HOLD:
      uiLedSet(PRO_LED,1);
      break;
    }
  }
  
  
  return 0;
}


