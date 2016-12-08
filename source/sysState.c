#include <aducm360.h>
#include "config.h"
#include "sysState.h"
#include "haladucm360.h"
#include "param.h"
#include "mbregs.h"
static sys_state_t sysState;

static int16_t opMode, opModeOld;
static uint16_t sysError = 0;
static uint16_t runCycles = 0;
static uint16_t ranCycles = 0;
static uint32_t ranSecs;
static uint32_t counter5ms;
static uint8_t ranDays,ranHours,ranMins;
static int16_t ens;
static uint16_t sysInfo = 0x0;
static double setPoint = 0;
static double tsp,gsp,bsp;
static int16_t ald,i1f,o1f,o1h,o1e;
static int16_t o2f;
static uint32_t o2EnabledSecs;

static uint8_t op1,op2;
//static uint8_t opMan;
static uint8_t di1;
static uint8_t opDir = DIR_UNKNOW;
static bool autoTuning;
static uint16_t alarmMask,alarmState, alarmOffCycles;
static uint16_t sysStatus = 0;
static int16_t sensorType;
static uint8_t modbusAddr;
static double pv;

static double proSpStep;
static uint16_t nofProSteps,nofProStepsRan;
static int16_t rrt,tms,tsh;
static uint32_t proStartTime;
static double proStartPv;
static uint8_t prosState;

int16_t alf,all,alh;
int16_t spl,sph;

static uint32_t atuneCounter = 0;

#define TEMP_SAMPLE_TIME        1000/CT2SEC
#define NOF_TEMP_HISTORY        8
static double tempHistory[NOF_TEMP_HISTORY];
uint32_t lastTempSample;
uint8_t tempCycles = 0;
uint8_t dirCycles = 0;

uint16_t mbByteSwap(uint16_t v)
{
  uint16_t vret;
  vret = ((v&0xff)<<8) | (v >> 8);

  return vret;
}

void sysResetATTimer()
{
  atuneCounter = 0;
}

uint32_t sysSetATTimer(uint32_t ct)
{
  uint32_t sec;
  sec = (ct - atuneCounter)/200;
  atuneCounter = ct;
  return sec;
}


void sysSetAlarm(uint8_t code)
{
  if(code < 16) alarmState |= (1 << code);
}

void sysClearAlarm(uint8_t code)
{
  if(code < 16) alarmState &= ~(1 << code);
}

void sysEnableAlarm(uint8_t code)
{
  if(code < 16) alarmMask |= (1 << code);
}

void sysDisableAlarm(uint8_t code)
{
  if(code < 16) alarmMask &= (1 << code);
}

uint8_t sysGetAlarm(uint8_t code)
{
  return (alarmState & (1 << code))==0?0:1;
}

void sysSetError(uint8_t code)
{
  if(code < 16) sysError |= (1 << code);
}

uint8_t sysGetError(uint8_t code)
{
  return (sysError & (1 << code))?1:0;
}


uint8_t isSysAlarm()
{
  return (alarmState == 0)?0:1;
}

uint8_t isSysError()
{
  
  return 0;
}

void sysSetOpMode(int16_t mode)
{
  opModeOld = opMode;
  opMode = mode;
  sysInfo |= MODE_ALTERED;
  
  switch(opMode){
    case modTCR:
      op1 = 0;
      readFloatParamByIndex(TSP,&setPoint);
      break;
    case modGE1:
      op1 = 0;
      readFloatParamByIndex(GSP,&setPoint);
      break;
    case modGE2:
      op1 = 0;
      readFloatParamByIndex(GSP,&setPoint);
      break;
    case modATO:
      op1 = 0;
      readFloatParamByIndex(TSP,&setPoint);
      sysResetATTimer();
      break;
    case modPRO:
      readFloatParamByIndex(TSP,&setPoint);
      //setPoint = 0;
      //writeIntParamByIndex(SP1,0);
      op1 = 0;
      prosState = PROS_IDLE;
      break;
    case modSWM:
      op1 = 0;
      break;    
  }
  sysSetParam(MOD,mode);
}

void sysSetProStart(uint32_t ct)
{
  //proStartTime = ct;
  proStartTime = ranSecs;
  nofProSteps = rrt*60/RRT_PERIOD;
  proSpStep = (float)(setPoint - pv)/(float)(nofProSteps);
  proStartPv = pv;
  sysSetSetpoint(proStartPv + proSpStep);
  prosState = PROS_RUN;
}

void sysSetProStop()
{
  prosState = PROS_IDLE;
}
uint8_t sysProPoll(uint32_t ct)
{
  uint8_t step = (ranSecs - proStartTime)/RRT_PERIOD;
  if(prosState == PROS_RUN){
    if(step <= nofProSteps){
      sysSetSetpoint(proStartPv + proSpStep*step);
    }else{
      if(tms > 0){
        if(((ranSecs - proStartTime)/60) > (rrt + tms)){
  //        op1 = 0;
          prosState = PROS_IDLE;
        }
      }
    }
    return 1;
  }
  return 0;
}
      
void sysSetProPause()
{
  prosState = PROS_HOLD;
}

uint8_t sysGetProState()
{
  return prosState;
}

uint8_t sysGetMBAddress()
{
  return modbusAddr;
}

void sysSetMBAddress(uint8_t v)
{
  if((v > 0) && (v <= 240)){
    modbusAddr = v;
    writeIntParamByIndex(ADR,modbusAddr);
    mbReset();
  }
}
uint8_t sysGetBaudrate()
{
  int16_t kv;
  readIntParamByIndex(BAU,&kv);
  return (uint8_t)kv;
}
void sysSetBaudrate(uint8_t v)
{
  writeIntParamByIndex(BAU,v);
  mbReset();
}
uint8_t sysGetParity()
{
  int16_t kv;
  readIntParamByIndex(PAR,&kv);
  return (uint8_t)kv;
}
void sysSetParity(uint8_t v)
{
  writeIntParamByIndex(PAR,v);
  mbReset();
}

void sysReturnOpMode()
{
  
  switch(opMode){
  case modMAN:
    op1 = 0;
    break;
  case modATO:
    op1 = 0;
    break;
  }
  
  opMode = opModeOld;
  sysInfo |= MODE_ALTERED;
}

int16_t sysGetOpMode()
{
  return opMode;
}

void sysSetSetpoint(double sp)
{
  setPoint = sp;
  sysInfo |= SP_ALTERED;
  writeFloatParamByIndex(SP1,sp);
}

double sysGetSetpoint()
{
  double v = setPoint;
  return v;
}

uint16_t resetSysInfo(uint8_t bit)
{
  sysInfo &= ~bit;
  return sysInfo;
}

uint8_t sysOpmodeAltered()
{
  return (sysInfo & MODE_ALTERED);
}

uint8_t sysSpAltered()
{
  return (sysInfo & SP_ALTERED);
}

uint8_t isSetpointAltered()
{
  return (sysInfo & SP_ALTERED);
}

void sysResetAlarm()
{
    alarmState=alarmOffCycles=0;
}
void sysClearError(uint8_t code)
{
  sysError &= ~(1 << code);
}

void sysResetError()
{
  sysError = 0;
}

void sysIncRunCycle()
{
  runCycles++;
  if(alarmOffCycles > 0) alarmOffCycles--;
}

void sysSetRunCycle(uint16_t cycle)
{
  runCycles = cycle;
}


void sysSetOP1Output(double v)
{
  // todo : control OP1 to v
  //int16_t o1f,o1h,o1e;
  //int16_t pv,sp;
  //float fv;
  //uint8_t duty = 0;
  readIntParamByIndex(O1F,&o1f);
  readIntParamByIndex(O1H,&o1h);
  readIntParamByIndex(O1E,&o1e);
  
  switch(o1f){
  case O1F_NON:
    op1 = 0;
    break;
  case O1F_HPI:
  case O1F_CPI:
    op1 = (uint8_t)v;
    break;
  case O1F_HNF:
//    readIntParamByIndex(TPV,&pv);
//    readIntParamByIndex(SP1,&sp);
    if(pv <= setPoint) op1 = 100;
    else op1 = 0;
    break;
  case O1F_HPR:
  case O1F_CNF:
  case O1F_CPR:
    break;
  }
  
  if(op1 == 0){
    halSetPwmIoMode(0);
  }else{
    halSetPwmIoMode(1);
    
  }
  
  //halSetPwmDuty(op1);
  writeScaledIntParamByIndex(MV1,op1);
}

uint16_t sysGetPwmDuty()
{
  return op1;
}


uint8_t sysGetOP1Output()
{
  return op1;
}

void sysSetOP2()
{
  int16_t o2f;
  //uint8_t updateOutput = 1;
  int16_t kv;
  readIntParamByIndex(O2F,&o2f);
 
  switch(o2f){
  case O2F_NONE:
    break;
  case O2F_ALA:
    if(isSysAlarm()){
      op2 = 1;
    }else{
      op2 = 0;
    }
    break;
  case O2F_TIM:
    if(o2EnabledSecs == 0)
      o2EnabledSecs = ranSecs;
    if(pv >= setPoint){
      if((ranSecs - o2EnabledSecs) < tms*60) 
        op2 = 1;
      else 
        op2 = 0;
    }
    break;
  case O2F_COM:
    //readIntParamByIndex(MV2,&kv);
    //op2 = kv;
    break;
  case O2F_DLL:
    if(((setPoint - all) < pv) && ((setPoint + alh) > pv))
      op2 = 1;
    else
      op2 = 0;
    break;
  case O2F_UP:
    if(opDir == DIR_UP){
      op2 = 1;
    }else{
      op2 = 0;
    }
    break;
  case O2F_DN:
    if(opDir == DIR_DN){
      op2 = 1;
    }else{
      op2 = 0;
    }
    break;
  case O2F_EON:
    if(isSysAlarm()) op2 = 1;
    else op2 = 0;
    break;
  case O2F_EOF:
    if(isSysAlarm()) op2 = 0;
    break;
  }
  
  halSetDigOut(op2);
}

uint8_t sysGetOP2Output()
{
  return op2;
}

void sysSetManOP1(uint8_t v)
{
  op1 = v;
}

uint8_t sysGetManOP1()
{
  return op1;
}

void sysIncManOP1()
{
  if(op1 <=100) op1++;
}

void sysDecManOP1()
{
  if(op1 > 0) op1--;
}

void sysArmManOP1()
{
  if(op1 == 0){
    halSetPwmIoMode(0);
  }else{
    halSetPwmIoMode(1);
  }
  
  //halSetPwmDuty(op1);
  writeIntParamByIndex(MV1,op1);

}

void sysArmManOP2()
{
  
}

void sysSetDI1(uint8_t v)
{
  
}

uint8_t sysGetDI1()
{
  di1 = halReadDigIn(0);
  writeIntParamByIndex(DI1,di1);
  return di1;
}

void sysCheckDI1()
{
  int16_t kv;
  readIntParamByIndex(I1F,&kv);
  
  switch(kv){
  case I1F_NONE:
    break;
  case I1F_GM1:
    // switch to GE1
    //sysSetOpMode(modGE1);
    break;
  case I1F_GM2:
    //sysSetOpMode(modGE2);
    break;
  case I1F_PRO:
    //sysSetOpMode(modPRO);
    break;
  case I1F_MAU:
    //sysSetOpMode(modMAN);
    break;
  case I1F_SWM:
    if(sysGetOpMode() == modSWM){
      sysSetOP1Output(di1*100);
    }
    break;
  case I1F_TIM:
    //sysSetOpMode(modTIM);
    if(di1 == 1){
      o2EnabledSecs = ranSecs;
    }
    break;
  }
}

void sysArmDI1ToOP1()
{
  
}

uint8_t sysGetSensorType()
{
  return sensorType;
}



void sysSetChainState(uint8_t state)
{
  if(state == 1){
    halSetDigOut(0);
  }else{
    halClrDigOut(0);
  }
}

uint32_t sysGetTimer()
{
  return counter5ms;
}

void sysIncCounter()
{
  counter5ms++;
}

void sysSetPv(double v)
{
  pv = v;
  writeFloatParamByIndex(TPV,pv);
}

double sysGetPv()
{
  return pv;
}

// this routine called periodically to check system constrains
int16_t sysProc(uint32_t ct)
{
  uint32_t sec = ct/CT2SEC;
  if((sec % 60) == 0){
    writeParams();
  }
  return 0;
}

void sysStateReset()
{
  
}

void sysSetStatus(uint8_t sta)
{
  sysStatus |= (1 << sta);
}

void sysClearStatus(uint8_t sta)
{
  sysStatus &= ~(1 << sta);
}

uint8_t sysGetStatus(uint8_t sta)
{
  return (sysStatus & (1 << sta))==0?0:1;
}

void sysCheckState()
{
  
  switch(alf){
  case ALF_DHI:
    if(pv > (setPoint + alh)){
      sysSetAlarm(ALF_DHI);
    }else{
      sysClearAlarm(ALF_DHI);
    }
    break;
  case ALF_DLO:
    if(pv < (setPoint - all)){
      sysSetAlarm(ALF_DLO);
    }else{
      sysClearAlarm(ALF_DLO);
    }
    break;
  case ALF_DBH:
    if((pv > (setPoint+alh)) || (pv < (setPoint - all))){
      sysSetAlarm(ALF_DBH);
    }else{
      sysClearAlarm(ALF_DBH);
    }
    break;
  case ALF_DBL:
    if((pv < (setPoint + alh)) && (pv > (setPoint - all))){
      sysSetAlarm(ALF_DBL);
    }else{
      sysClearAlarm(ALF_DBL);
    }
    break;
  case ALF_PVH:
    if(pv > alh){
      sysSetAlarm(ALF_PVH);
    }else{
      sysClearAlarm(ALF_PVH);
    }
    break;
  case ALF_PVL:
    if(pv < all){
      sysSetAlarm(ALF_PVL);
    }else{
      sysClearAlarm(ALF_PVL);
    }
    break;
  case ALF_PBH:
    if(pv > alh && pv < alf){
      sysSetAlarm(ALF_PBH);
    }else{
      sysClearAlarm(ALF_PBH);
    }
    break;
  case ALF_PBL:
    if(pv < alh && pv > all){
      sysSetAlarm(ALF_PBL);
    }else{
      sysClearAlarm(ALF_PBL);
    }
    break;
  }
  
  // check run cycles
  if(ranCycles > 0){
    writeIntParamByIndex(ERR,alarmState);
  }
  
  if(sysGetAlarm(ALF_PVL)){
    sysSetStatus(STA_LOW_LIMIT);
  }else{
    sysClearStatus(STA_LOW_LIMIT);
  }
  
  if(sysGetAlarm(ALF_PVH)){
    sysSetStatus(STA_HI_LIMIT);
  }else{
    sysClearStatus(STA_HI_LIMIT);
  }
  
  // check for time state
  if(ens != 0){
    if(ranDays < ens){
      sysSetStatus(STA_ENS);
    }else{
      sysClearStatus(STA_ENS);
    }
    writeIntParamByIndex(ENS,ens - ranDays);
  }
  
  // DLL
  
  
  // fire alarm
  
}

uint8_t sysStateLocked()
{
  
  return 0;
}


static uint8_t key;
uint16_t sysGetParam(uint16_t index)
{
  int16_t kv;

  if(index == key)
    kv = 3;
  switch(index){
  case CMD:
    kv = 0;
    break;
  default:
    readIntParamByIndex(index,&kv);
  }
  return kv;
}

void sysSetParam(uint16_t index, uint16_t v)
{
  int16_t res = (int16_t)v;
  int16_t kv1,kv2;
  switch(index){
  case MOD:
    sysInfo |= MODE_ALTERED;
    opMode = v;
    writeIntParamByIndex(index,v);      
    break;
  case TSP:
  case GSP:
  case BSP:
    readScaledIntParamByIndex(index,&kv1);
    if((kv1 <= sph) && (kv1 >= spl)){
      writeIntParamByIndex(index,v);      
      sysInfo |= MODE_ALTERED;
    }
    break;
  case RRT:
    rrt = v;
    writeIntParamByIndex(index,v);      
    break;
  case TMS:
    tms = v;
    writeIntParamByIndex(index,v);      
    break;
  case IN:
    sensorType = v;
    writeIntParamByIndex(IN,v);
    break;
  case SPL:
    //spl = v;
    writeIntParamByIndex(index,v);
    readScaledIntParamByIndex(index,&spl);
    break;
  case SPH:
    sph = v;
    writeIntParamByIndex(index,v);
    readScaledIntParamByIndex(index,&sph);
    break;
  case ALH:
    alh = v;
    writeIntParamByIndex(index,v);
    readScaledIntParamByIndex(index,&alh);
    break;
  case ALL:
    all = v;
    writeIntParamByIndex(index,v);
    readScaledIntParamByIndex(index,&all);
    break;
  case ALF:
    alf = v;
    writeIntParamByIndex(index,v);
    break;
  case ALD:
    ald = v;
    writeIntParamByIndex(index,v);
    break;
  case I1F:
    i1f = v;
    writeIntParamByIndex(index,v);
    break;
  case O1F:
    o1f = v;
    writeIntParamByIndex(index,v);
    break;
  case O2F:
    o2f = v;
    writeIntParamByIndex(index,v);
    break;
  default:
    writeIntParamByIndex(index,v);
  }

}

int16_t sysPeriodicPoll(uint32_t ct)
{
  counter5ms = ct;
  
  if((ct - lastTempSample) > TEMP_SAMPLE_TIME){
    lastTempSample = ct;
    bool isMax=true, isMin=true;
    double v;
    int8_t i;
    for(i=NOF_TEMP_HISTORY-1;i>=0;i--){
      v = tempHistory[i];
      if(isMax) isMax = pv > v;
      if(isMin) isMin = pv < v;
      tempHistory[i+1] =  tempHistory[i];
    }
    tempHistory[0] = pv;
    if(tempCycles < 8){
      tempCycles++;
    }else{
      if(isMax){
        if(opDir == DIR_DN) opDir = DIR_UP;
        if(opDir == DIR_UNKNOW){
          opDir = DIR_UP;
          dirCycles++;
        }
      }
      if(isMin){
        if(opDir == DIR_UP) opDir = DIR_DN;
        if(opDir == DIR_UNKNOW){
          opDir = DIR_DN;
          dirCycles++;
          sysIncRunCycle();
        }
      }
      
    }
  }
  
  
  
  return (counter5ms/CT2SEC);
}

void sysNotifyParamAltered()
{
  uint8_t i;
  // save settings & notify param change

  for(i=0;i<NOF_PARAM;i++){
    if(paramIsModified(i) == 1){
      paramClearModifyFlag(i);
      switch(i){
      case LOK:
        break;
      case MOD:
        break;
      case TSP:
        break;
      case GSP:
        break;
      case BSP:
        break;
      case ALL:
        break;
      case ALH:
        break;
      case RRT:
        break;
      case CMD:
        break;
      case PIDP:
        break;
      case PIDI:
        break;
      case PIDD:
        break;
      case SPL:
        break;
      case SPH:
        break;
      case ADR:
        break;
      case LED:
        break;
      case IN:
        break;
      case TSH:
        break;
      case O1F:
        break;
      case O1H:
        break;
      case O1E:
        break;
      case PLL:
        break;
      case PLH:
        break;
      case ALF:
        break;
      case ALD:
        break;
      case O2F:
        break;
      case I1F:
        break;
      case BAU:
        break;
      case PAR:
        break;
        
      }
    }
  }
  
  writeParams();
  
}

void sysAlterParams(uint8_t index, int16_t v)
{
  
}

void sysAddSeconds()
{
  ranSecs++;
}



void sysInit()
{
  opDir = DIR_UP;
  opMode = opModeOld = modTCR;
  ranSecs = 0;
  ranDays = ranHours = ranMins = 0;
 
  alarmMask = 0xffff;
  alarmState = 0x0;
  alarmOffCycles = 0;
  counter5ms = 0;
  
  readFloatParamByIndex(TSP,&tsp);
  readFloatParamByIndex(GSP,&gsp);
  readFloatParamByIndex(BSP,&bsp);
  op1 = op2 = 0;
  setPoint = tsp;
  
  readIntParamByIndex(ADR,(int16_t*)&modbusAddr);
  if(modbusAddr == 250){
    sysSetChainState(0);
  }else{
    sysSetChainState(0);
  }
  
  readIntParamByIndex(ENS,(int16_t*)&ens);
  readIntParamByIndex(IN,(int16_t*)&sensorType);
  readIntParamByIndex(RRT,(int16_t*)&rrt);
  readIntParamByIndex(TMS,(int16_t*)&tms);
  readIntParamByIndex(TSH,(int16_t*)&tsh);
  readIntParamByIndex(TMS,(int16_t*)&tms);
  readIntParamByIndex(ALF,(int16_t*)&alf);
  readIntParamByIndex(ALL,(int16_t*)&all);
  readIntParamByIndex(ALH,(int16_t*)&alh);
  /*
  readIntParamByIndex(ENS,(int16_t*)&all);
  readIntParamByIndex(ENS,(int16_t*)&all);
  readIntParamByIndex(ENS,(int16_t*)&all);
  readIntParamByIndex(ENS,(int16_t*)&all);
  */
  
}

