#include <aducm360.h>
#include "config.h"
#include "eeParams.h"
#include "param.h"
#include "sysState.h"

const param_map_t paramMap[]=
{
  {"SYS",SYS,HEADING,0,0,0,1,0},
  {"TPV",TPV,0,1,-3000,9990,10,0},
  {"SP1",SP1,0,1,-3000,9990,10,0},
  {"MV1",MV1,0,1,0,1000,10,0},
  {"ALL",ALL,100,1,-3000,9990,10,0},
  {"ALH",ALH,800,1,-3000,9990,10,0},
  {"MOD",MOD,0,0,0,4,1,0},
  {"ADR",ADR,250,0,1,250,1,0},
  {"STA",STA,0,0,0,0xffff,1,0},
  {"MV2",MV1,0,1,0,10,10,0},
  {"ERR",ERR,0,0,0,0xffff,1,0},
  {"DI1",DI1,0,0,0,1,1,0},
  {"-V-",VV,0,1,0,300,1,0},
  {"-A-",AA,0,1,0,5,1,0},
  {"-P-",PP,0,1,0,999,1,0},
  {"KWH",KWH,0,1,-99,999,1,0},
  {"-ID",ID,0,0,0,0xffff,1,0},
  {"ENS",ENS,0,0,0,0xffff,1,0},
  {"ENP",ENP,0,0,0,0xffff,1,0},
  {"LED",LED,0,0,0,8,1,0},
  {" IN",IN,0,0,0,2,1,0},
  {"TSH",TSH,0,1,-5000,5000,10,0},
  {"PLL",PLL,0,0,0,99,1,0},
  {"PLH",PLH,1,0,1,100,1,0},
  {"ALF",ALF,3,0,0,8,1,0},
  {"RRT",RRT,0,0,0,999,1,0},
  {"CMD",CMD,0,0,0,999,1,0},
  {"-P-",PIDP,0,3,0,9999,1,0},
  {"-I-",PIDI,0,3,0,9999,1,0},
  {"-D-",PIDD,0,3,0,9999,1,0},
  {"O1F",O1F,1,0,0,6,1,0},
  {"O1H",O1H,10,1,-5000,5000,10,0},
  {"O1E",O1E,0,0,0,2,1,0},
  {"ALD",ALD,0,0,0,3,1,0},
  {"O2F",O2F,1,0,0,8,1,0},
  {"I1F",I1F,1,0,0,6,1,0},
  {"CIF",CIF,0,0,0,3,1,0},
  {"LOK",LOK,0,0,0,666,666,0},
  {"RRT",RRT,0,0,0,999,1,0},
  {"TMS",TMS,250,0,1,240,1,0},
  {"BAU",BAU,2,0,0,3,1,0},
  {"PAR",PAR,0,0,0,2,1,0},
  {"TSP",TSP,300,1,-3000,9990,10,0},
  {"GSP",GSP,50,1,-3000,9990,10,0},
  {"BSP",BSP,0,0,0,5,1,0},
  {"SPL",SPL,0,1,-3000,9990,10,0},
  {"SPH",SPH,0,1,-3000,9990,10,0},
  {"ATU",ATU,0,0,0,2,1,0},
  {"DSP",DSPC,0,0,0,2,1,0},
  {"DSP",DSPO,0,0,0,2,1,0},
  {"PDM",PIDM,0,0,0,2,1,0},
  {"MOD",MODS,0,0,0,2,1,0},
  {"PRO",PROS,0,0,0,0,0,0},       // profile state
};


static param_map_t params[NOF_PARAM];
sys_params_t sysParams;
const unsigned char view_order[]={TPV,SP1,TSP,GSP,MV1,MV2,DI1,CIF,STA,ERR,ID,ENP,VV,AA,PP,KWH};
const unsigned char set_order[]={LOK,MOD,TSP,GSP,ALL,ALH,RRT,CMD,BSP,PIDP,PIDI,PIDD,SPL,SPH};
const unsigned char cfg_order[]={ADR,LED,IN,TSH,O1F,O1H,O1E,PLL,PLH,ALF,ALD,O2F,I1F,BAU,PAR};

static uint8_t currentIndex;
static uint8_t currentParamIndex;
static uint8_t showTarget;
static uint8_t currentOrder;
static uint8_t ledDispmap[] = {TPV,TSP,GSP,MV1,MV2};

void paramInit(void)
{
  int16_t kv;
  
  loadParams();
  sysInit();
  
  readIntParamByIndex(MOD,&kv);
  if((kv == modATO) || (kv == modMAN)){
    kv = modTCR;
  }
  sysSetOpMode(kv);
  sysSetRunCycle(0);
  sysSetOP1Output(0);
  sysSetOP2(0);
  
  
  currentParamIndex = TPV;
  currentIndex = 0;
  showTarget = SHOW_STRING;
  currentOrder = ODR_IDLE;
}

void paramSetActiveOrder(uint8_t ord)
{
  int16_t kv;
  switch(currentOrder){
  case ODR_IDLE:
    // in idle mode, check if in modMAN or modATO mode
    kv = sysGetOpMode();
    switch(kv){
    case modNONE:
      break;
    case modTCR:
      break;
    case modMAN:
      // return to previous mode
      sysReturnOpMode();
      break;
    case modATO:
      // return to previous mode
      sysReturnOpMode();
      break;
    }
    break;
  case ODR_VIEW:
    break;
  case ODR_SET:
    break;
  case ODR_CFG:
    break;
  }

  currentIndex = 0;
  switch(ord){
  case ODR_VIEW:
    currentParamIndex = view_order[0];
    break;
  case ODR_SET:
    currentParamIndex = set_order[0];
    break;
  case ODR_CFG:
    currentParamIndex = cfg_order[0];
    break;
  }
  currentOrder = ord;
}

uint8_t paramGetCurrentOrder()
{
  return currentOrder;
}

void paramMoveAdv()
{
  uint8_t keySize;
  currentIndex++;
  showTarget = SHOW_STRING;
  switch(currentOrder){
  case ODR_IDLE:
    currentOrder = ODR_VIEW;
    showTarget = SHOW_STRING;
    break;
  case ODR_VIEW:
    keySize = sizeof(view_order);
    if(currentIndex == keySize) currentIndex = 0;
    currentParamIndex = view_order[currentIndex];
    break;
  case ODR_SET:
    keySize = sizeof(set_order);
    if(currentIndex == keySize) currentIndex = 0;
    currentParamIndex = set_order[currentIndex];
    break;
  case ODR_CFG:
    keySize = sizeof(cfg_order);
    if(currentIndex == keySize) currentIndex = 0;
    currentParamIndex = cfg_order[currentIndex];
    break;
  }
}

void paramValueInc()
{
  switch(currentOrder){
  case ODR_IDLE:
    /*
    switch(sysGetOpMode()){
    case modMAN:
      sysIncManOP1();
      break;
    }
    */
    break;
  case ODR_VIEW:
    showTarget = (showTarget==SHOW_VALUE)?SHOW_STRING:SHOW_VALUE;
    break;
  case ODR_SET:
  case ODR_CFG:
    if(showTarget == SHOW_STRING){
      showTarget = SHOW_VALUE;
    }else{
      int16_t ulimit = params[currentParamIndex].uLimit;
      switch(currentParamIndex){
      case TSP:
      case GSP:
      case BSP:
        ulimit = params[SPH].uLimit;
        break;
      }
      params[currentParamIndex].val++;
      if(params[currentParamIndex].val > ulimit)
        params[currentParamIndex].val--;
      params[currentParamIndex].modified = 1;
    }
    break;
  case ODR_MAN:
    sysIncManOP1();
    break;
  }
}

void paramValueDec()
{
  switch(currentOrder){
  case ODR_IDLE:
    /*
    switch(sysGetOpMode()){
    case modMAN:
      sysDecManOP1();
      break;
    }
    */
    break;
  case ODR_VIEW:
    showTarget = (showTarget==SHOW_VALUE)?SHOW_STRING:SHOW_VALUE;
    break;
  case ODR_SET:
  case ODR_CFG:
    int16_t llimit = params[currentParamIndex].lLimit;
    if(showTarget == SHOW_STRING){
      showTarget = SHOW_VALUE;
    }else{
      switch(currentParamIndex){
      case TSP:
      case GSP:
      case BSP:
        llimit = params[SPL].lLimit;
      }
      params[currentParamIndex].val--;
      if(params[currentParamIndex].val < llimit)
        params[currentParamIndex].val++;
      params[currentParamIndex].modified = 1;
    }
    break;
  case ODR_MAN:
    sysDecManOP1();
    break;
  }
}

int16_t paramGetDisplayString(char *str)
{
  uint8_t showIndex;
  int16_t kv;
  switch(currentOrder){
  case ODR_IDLE:
    switch(sysGetOpMode()){
    case modMAN:
      sprintf(str,"%03d",sysGetManOP1());
      break;
    default:
      showIndex = params[LED].val;
      readScaledIntParamByIndex(ledDispmap[showIndex],&kv);
      sprintf(str,"%03d",kv);    
    }
    break;
  case ODR_VIEW:
    currentParamIndex = view_order[currentIndex];
    if(showTarget == SHOW_STRING){
      sprintf(str,"%s",params[currentParamIndex].disp);
    }else{
      readScaledIntParamByIndex(currentParamIndex,&kv);
      sprintf(str,"%03d",params[kv]);
    }
    break;
  case ODR_SET:
    currentParamIndex = set_order[currentIndex];
    if(showTarget == SHOW_STRING){
      sprintf(str,"%s",params[currentParamIndex].disp);
    }else{
      readScaledIntParamByIndex(currentParamIndex,&kv);
      sprintf(str,"%03d",kv);
    }
    break;
  case ODR_CFG:
    currentParamIndex = cfg_order[currentIndex];
    if(showTarget == SHOW_STRING){
      sprintf(str,"%s",params[currentParamIndex].disp);
    }else{
      readScaledIntParamByIndex(currentParamIndex,&kv);
      sprintf(str,"%03d",kv);
    }
    break;
  case ODR_MAN:
      sprintf(str,"%03d",sysGetManOP1());
    break;
  case ODR_ATO:
      sprintf(str,"%03d",(int16_t)sysGetPv());
    break;
  }

  return 1;
}

void paramSetShowTarget(uint8_t target)
{
  showTarget = target;
}

uint8_t paramGetShowTarget()
{
  return showTarget;
}

int16_t readScaledIntParamByIndex(uint16_t index,int16_t *val)
{
  uint8_t i;
  for(i=0;i<NOF_PARAM;i++){
    if(params[i].mbRegOrder == index){
      double v = (double)params[i].val;
      int16_t j = params[i].digits;
      while(j--){
        v /= 10.;
      }
      *val = (int16_t)lround(v);
      return 1;
    }
  }
  return 0;
}

int16_t readIntParamByIndex(uint16_t index,int16_t *val)
{
  uint8_t i;
  for(i=0;i<NOF_PARAM;i++){
    if(params[i].mbRegOrder == index){
      *val = params[i].val;
      return 1;
    }
  }
  return 0;
}

int16_t readFloatParamByIndex(uint16_t index,double *val)
{
  uint8_t i;
  for(i=0;i<NOF_PARAM;i++){
    if(params[i].mbRegOrder == index){
      double v = (double)params[i].val;
      int16_t j = params[i].digits;
      while(j--){
        v /= 10.;
      }
      *val = v;
      return 1;
    }
  }
  return 0;
}

int16_t writeScaledIntParamByIndex(uint16_t index,int16_t val)
{
  uint8_t i;
  for(i=0;i<NOF_PARAM;i++){
    if(params[i].mbRegOrder == index){
      uint8_t j = params[i].digits;
      double v = val;
      while(j--){
        v *= 10.;
      }
      params[i].val = (int16_t)lround(v);
      return 1;
    }
  }
  return 0;
}

int16_t writeIntParamByIndex(uint16_t index,int16_t val)
{
  uint8_t i;
  for(i=0;i<NOF_PARAM;i++){
    if(params[i].mbRegOrder == index){
      params[i].val = val;
      return 1;
    }
  }
  return 0;
}

uint8_t paramIsModified(uint8_t index)
{
  return params[index].modified;
}

void paramClearModifyFlag(uint8_t index)
{
  params[index].modified = 0;
}

int16_t writeFloatParamByIndex(uint16_t index, double val)
{
  uint8_t i;
  for(i=0;i<NOF_PARAM;i++){
    if(params[i].mbRegOrder == index){
      int16_t j = params[i].digits;
      double v = val;
      if(params[i].digits > 0){
        do{
          v *= 10;
        }while(--j);
      }
      params[i].val = (int16_t)v;
      return 1;
      
    }
  }
  return 0;
}

void writeParams()
{
  //memcpy((char*)data,(char*)sysKeys,NOFKEY*20);
  FeePErs(FEE_START_ADDR);
  FeePErs(FEE_START_ADDR+0x200);
  WriteToFlash((unsigned long*)params,FEE_START_ADDR,sizeof(params));  

  //memcpy((char*)data,(char*)&sysParams,sizeof(sysParams));
  FeePErs(FEE_SYS_PARAM_ADDR);
  WriteToFlash((unsigned long*)&sysParams,FEE_SYS_PARAM_ADDR,sizeof(sysParams));    
    
}

void loadDefaultSettings()
{
    memcpy((char*)params,(char*)paramMap,NOF_PARAM*sizeof(param_map_t));
    writeParams();
}

void loadParams()
{
  char data[1024];
  char *dptrsrc, *dptrdest;
  
  int i;
  //memset(data,0,1024);
//  ReadFromFlash((unsigned long*)data,FEE_ADDRESS,1024);
  ReadFromFlash((unsigned long*)params,FEE_START_ADDR,sizeof(params));
  //memcpy((char*)sysKeys,(char*)data,NOFKEY*20);
  
  if(params[0].val != HEADING){
    loadDefaultSettings();
  }
  
  ReadFromFlash((unsigned long*)&sysParams,FEE_SYS_PARAM_ADDR,sizeof(sysParams));
//  ReadFromFlash((unsigned long*)data,SYS_PARAM_ADDR,128);
//  memcpy((char*)&sysParams,(char*)data,sizeof(sysParams));
  if(sysParams.flag != HEADING){
    sysParams.flag = HEADING;
    sysParams.enp = 0;
    sysParams.ens = 0;
    sysParams.err = 0;
    sysParams.id = 100;
    sysParams.lock = 0;
    sysParams.runDate.days = 0;
    sysParams.runDate.hours = 0;
    sysParams.runDate.minutes = 0;
    sysParams.runDate.secs = 0;
    sysParams.runDate.tmr = 0;
    sysParams.lastWrite.days = 0;
    sysParams.lastWrite.hours = 0;
    sysParams.lastWrite.minutes = 0;
    sysParams.lastWrite.secs = 0;
    sysParams.lastWrite.tmr = 0;
    memcpy((char*)data,(char*)&sysParams,sizeof(sysParams));
    FeePErs(FEE_SYS_PARAM_ADDR);
    WriteToFlash((unsigned long*)data,FEE_SYS_PARAM_ADDR,128);    
  }

}

