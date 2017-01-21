#include <aducm360.h>
#include "keypad.h"
#include "param.h"
#include "pid.h"
#include "piduser.h"
#include "sensorif.h"
#include "userif.h"
#include "halAducm360.h"
#include "modOperation.h"
#include "sysState.h"

int16_t currMode;
double kp, ki, kd;
double pidIn, pidOut, pidSp;
bool pidTuning;


int16_t modOpPoll(uint32_t ct)
{
  int16_t opMode;
  int16_t ret;
  double fv,fv2;
  uint8_t newTemp = 0;
  uint8_t sensorErr = 0;
  
  if(sysStateLocked()) return 0;
  
  // poll temperature system
  if(sensorAcqDone()){
    if(sensorGetTemp(&fv,&fv2) == 1){
      // sensor error occurred, discard
      sysSetAlarm(SS_SENSOR);
      return 0;
    }
    pidIn = fv + fv2;
    sysSetPv(pidIn);
  }
  
  opMode = sysGetOldOpMode();
  int16_t oldm;
  if(sysOpmodeAltered()){
    oldm = sysGetOldOpMode();
    opMode = sysGetNewMode();
    fv = 0.0;
    switch(oldm){
    case modATO:
    case modMAN:
      uiLedClear(MAN_LED);
      uiSetDispFlash(0);
      break;
    case modPRO:
      uiLedClear(PRO_LED);
      break;
    }
    switch(opMode){
    case modTCR:
      readFloatParamByIndex(TSP,&fv);
      sysSetSetpoint(fv);
      break;
    case modGE1:
      readFloatParamByIndex(GSP,&fv);
      sysSetSetpoint(fv);
      break;
    case modGE2:
      break;
    case modATO:
      readFloatParamByIndex(TSP,&fv);
      sysSetSetpoint(fv);
      sysResetATTimer();
      uiLedSet(MAN_LED,0);
      uiSetDispFlash(1);
      break;
    case modPRO:
      readFloatParamByIndex(TSP,&fv);
      sysSetSetpoint(fv);
      //readFloatParamByIndex(TSP,&fv);
      break;
    case modMAN:
      sysSetManOP1(0);
      uiLedSet(MAN_LED,0);
    case modSWM:
      break;
    }
    resetSysInfo(MODE_ALTERED);
    sysConfirmOpMode();
  }
  
  uint8_t sysInfo;
  if(sysInfo = sysGetInfo()){
    if(sysInfo & PID_ALTERED){
      readFloatParamByIndex(PIDP,&kp);
      readFloatParamByIndex(PIDI,&ki);
      readFloatParamByIndex(PIDD,&kd);
      pidSetTunings(kp,ki,kd);
      sysClearInfo(PID_ALTERED);
    }
    if(sysInfo & SP_ALTERED){
      
      sysClearInfo(SP_ALTERED);
    }
  }

  if(sysSpAltered()){
    pidSp = sysGetSetpoint();
    resetSysInfo(SP_ALTERED);
    // reset run cycle
    sysSetRunCycle(0);
  }

  opMode = sysGetOpMode();    
  //enter operation mode
  switch(opMode){
  case modTCR:
  case modGE1:
    pidSp = sysGetSetpoint();
    pidCompute();
    sysSetOP1Output(pidOut);
    break;
  case modATO:
    pidSp = sysGetSetpoint();
    ret = pidATRuntime();
    if(ret == 1){
      kp = pidATGetKp();
      ki = pidATGetKi();
      kd = pidATGetKd();
      // for stability reason, divide kd by 10
      kd /= 10.;
      pidSetTunings(kp,ki,kd);
      writeFloatParamByIndex(PIDP,kp);
      writeFloatParamByIndex(PIDI,ki);
      writeFloatParamByIndex(PIDD,kd);
      writeParams();
      sysReturnOpMode();
    }
    if(sysSetATTimer(ct) > 1800){
      // return to tcr mode
      sysSetOpMode(modTCR);
      sysSetError(SS_ATE);
    }else{
      sysSetOP1Output(pidOut);
    }
    break;
  case modMAN:
    sysArmManOP1();
    break;
  case modGE2:
    break;
  case modSWM:
    sysArmDI1ToOP1();
    break;
  case modPRO:
    switch(sysGetProState()){
    case PROS_RUN:
      sysProPoll(ct);
      pidSp = sysGetSetpoint();
      pidCompute();
      sysSetOP1Output(pidOut);
      break;
    case PROS_HOLD:
      break;
    default:
      sysSetOP1Output(0);
    }
    break;
  }

// check DI state
  sysCheckDI1();
    
  sysCheckState();
  ret = sysGetOP2Output();
  sysSetOP2(ret);
  sysPeriodicPoll(ct);
  
  return 0;
}

void modOpInit()
{
  float fv;
  //readIntParamByIndex(MOD,&currMode);
  readFloatParamByIndex(PIDP,&kp);
  readFloatParamByIndex(PIDI,&ki);
  readFloatParamByIndex(PIDD,&kd);
  
  // initialize PID controller
  pidIn = pidOut = pidSp = 0.0;
  pidInit(&pidIn,&pidOut,&pidSp,kp,ki,kd,DIRECT);
  pidATuneInit(&pidIn,&pidOut);
  pidATSetControlType(AT_PID);
  double low,high;
  readFloatParamByIndex(PLL,&low);
  readFloatParamByIndex(PLH,&high);
  pidSetOutputLimits(low,high);
  pidSetMode(AUTOMATIC);
  
  //sysStateReset();
    uiInit();
  // start sensor interface
  sensorInit();
  
  
  
  
  switch(sysGetOpMode()){
  case modTCR:
    
    break;
  case modGE1:
    break;
  case modGE2:
    break;
  case modPRO:
    // write mode again to trigger mode altered
    sysSetOpMode(modPRO);
    break;
  case modSWM:
    break;
  }
  
  
}