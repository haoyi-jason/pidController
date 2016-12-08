#include <aducm360.h>
#include "config.h"
#include "piduser.h"
//#include "keymap.h"
#include "pid.h"
//#include "display.h"
//#include "ioControl.h"

// pid parameters
#define ATUNE_STEPS     50
#define ATUNE_NOISE     0.5
#define ATUNE_STARTVALUE        50
#define ATUNE_LOOKBACK  10


static uint8_t pidState;
double in,out,sp;
bool tuning;

int pidUserInit()
{
  int16_t val;
  float kp,ki,kd;
  readFltKeyByReg(PIDP,&kp);
  readFltKeyByReg(PIDI,&ki);
  readFltKeyByReg(PIDD,&kd);
  
  pidInit(&in,&out,&sp,kp,ki,kd,DIRECT);
  pidATuneInit(&in,&out);
  pidSetOutputLimits(0,100);
  pidSetMode(AUTOMATIC);
  pidState = PID_IDLE;
  tuning = false;
}

int pidUserPoll()
{
  int16_t ret;
  int16_t mod;
  float sp1;
  float temp;
  readFltKeyByReg(TPV,&temp);
  readIntKeyByReg(MOD,&mod);
  in = temp;

  //int ret = 0;
  
  switch(mod){
  case modTCR:
  case modGE1:
  case modGE2:

    readFltKeyByReg(SP1,&sp1);
    sp = sp1;
    pidCompute();
    //writeFltKeyByReg(MV1,out);
    ret = (int)out;
    break;
  case modATO:
    ret = pidATRuntime();
    if(ret == 1){
      pidState = PID_AUTO;
      double kp = pidATGetKp();
      double ki = pidATGetKi();
      double kd = pidATGetKd();
      pidSetTunings(kp,ki,kd);
      writeFltKeyByReg(PIDP,kp);
      writeFltKeyByReg(PIDI,ki);
      writeFltKeyByReg(PIDD,kd);
      //dispClrLed(AT_LED);
      pidSetSampleTime(100);
      
      // return to idle mode
      writeIntKeyByReg(MOD,modNONE);
    }else{
//      writeFltKeyByReg(MV1,out);
      ret = (int)out;
    }
    break;
  default:
    writeIntKeyByReg(MV1,0);
  }

  return ret;
}

int pidUserGetState()
{
  return pidState;
}

void pidUserSetState(uint8_t s)
{
  switch(pidState){
  case PID_IDLE:
    pidState = s;
    break;
  case PID_AUTO:
    pidState = s;
    break;
  case PID_TUNING:
    pidState = s;
    break;
  }

}

void pidUserSetIn(double v)
{
  in = v;
}

void pidUserSetSp(double v)
{
  sp = v;
}

double pidUserGetOut()
{
  return out;
}

void pidUserSetAuto()
{
  if(!pidATuneGetState()){
    pidState = PID_TUNING;
    out = ATUNE_STEPS;
    pidATuneInit(&in,&out);
    
    int16_t spVal;
    float fv;
    readIntKeyByReg(TSP,&spVal);
    writeIntKeyByReg(SP1,spVal);
    sp = (double)spVal;
    pidATSetNoiseBand(ATUNE_NOISE);
    pidATSetOutputStep(ATUNE_STEPS);
    pidATSetLoopbackSec((int)ATUNE_LOOKBACK);
  }  
}

void pidUserClrAuto()
{
  
}

