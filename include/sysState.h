#ifndef _SYSSTATE_
#define _SYSSTATE_
#include "config.h"


#define NOF_LOOKBACK    30
#define ATUN_TIMEOUT_MIN        20      // 20 minites timeout
#define RRT_PERIOD      10      // second

enum {
  DIR_UNKNOW,
  DIR_UP,
  DIR_DN
};

enum {
  SS_ATE,
};


enum sys_status_e{
  SS_DLO,
  SS_DHI,
  SS_HWARE,
  SS_SENSOR,
  SS_DO1,
  SS_ENS,
  SS_RSV1,
  SS_RSV2,
  SS_RSV3,
  SS_DLL,
  SS_UP,
  SS_DN,
  SS_PRO_RUN,
  SS_PRO_OFF,
  SS_PRO_IDLE,
};

enum alarm_drive_e{
  AD_NOR,
  AD_LTC,
  AD_HOLD,
  AD_SPH,
};

enum{
  MODE_ALTERED = 0x01,
  SP_ALTERED = 0x02
};

enum o1fFunc_e{O1F_NON,O1F_HPI,O1F_HNF,O1F_HPR,O1F_CPI,O1F_CNF,O1F_CPR};
enum o1eFunc_e{O1E_OFF,O1E_ON,O1E_KEP};


enum op2_function_e{
  O2F_NONE,
  O2F_ALA,
  O2F_TIM,
  O2F_COM,
  O2F_DLL,
  O2F_UP,
  O2F_DN,
  O2F_EON,
  O2F_EOF
};

enum i1f_function_e{
  I1F_NONE,
  I1F_GM1,
  I1F_GM2,
  I1F_PRO,
  I1F_MAU,
  I1F_SWM,
  I1F_TIM,
};

enum cif_function_e{
  CIF_NONE,
  CIF_PROST,
  CIF_PROSP,
  CIF_PROH,
};

enum cmd_function_e{
  CMD_RES,
  CMD_MAU,
  CMD_AT,
};

enum systemStatus_e{
  STA_LOW_LIMIT,
  STA_HI_LIMIT,
  STA_HW_ERR,
  STA_TC_ERR,
  STA_OP1_ERR,
  STA_ENS,
  STA_RSV1,
  STA_RSV2,
  STA_RSV3,
  STA_DLL_RUN,
  STA_HEAT_UP,
  STA_COOL_DN,
  STA_PRO_RUN,
  STA_PRO_OFF,
  STA_PRO_TOUT,
};
enum alfFunc_e{
  ALF_NONE,
  ALF_DHI,
  ALF_DLO,
  ALF_DBH,
  ALF_DBL,
  ALF_PVH,
  ALF_PVL,
  ALF_PBH,
  ALF_PBL
};

enum prosState_e{
  PROS_IDLE,
  PROS_RUN,
  PROS_HOLD,
};

typedef struct sys_state_s{
  uint8_t prevMode;
  uint32_t runInSec;
  uint32_t runInMin;
  uint32_t runInHour;
  uint32_t runInDays;
  uint32_t runInCycles; 
  uint8_t curDirection;
  uint8_t almOffCycles;
  uint32_t autoRunStartSec;
  bool autoTuning;
  bool modeChanged;
  uint16_t proRunSteps;
  uint16_t proStepsToRun;
  uint32_t proStartSec;
  float proSpStep;
  bool proRun;
  uint8_t prevProState;
  uint8_t lastSaveMin;
  float rrtInitTemp;
  float inpurRecord[NOF_LOOKBACK+1];
  int8_t peakType;
  uint16_t alarmState;
  uint16_t alarmMask;
  uint8_t alarmOccurred;
  uint16_t status;
  uint32_t o2fTimerSec;
  int8_t cyclesRan;
  int8_t cyclesToRun;
  uint16_t systemStatus;
}sys_state_t;

void sysSetOpMode(int16_t mode);
void sysReturnOpMode();
int16_t sysGetOpMode();
void sysSetSetpoint(double sp);
double sysGetSetpoint();
uint8_t sysStateLocked();
void sysSetOP2();
uint16_t sysGetPwmDuty();
void sysIncCounter();
void sysIncManOP1();
void sysDecManOP1();
void sysSetPv(double v);
double sysGetPv();

void sysCheckDI1();

void sysCheckState();

void sysResetATTimer();
uint32_t sysSetATTimer(uint32_t ct);

uint8_t sysGetMBAddress();
void sysSetMBAddress(uint8_t v);
uint8_t sysGetBaudrate();
void sysSetBaudrate(uint8_t v);
uint8_t sysGetParity();
void sysSetParity(uint8_t v);

uint16_t sysGetParam(uint16_t index);
void sysSetParam(uint16_t index, uint16_t v);

void sysSetError(uint8_t code);
void sysClearError(uint8_t code);
void sysResetError();
uint8_t sysGetError(uint8_t code);

int16_t sysPeriodicPoll(uint32_t ct);

uint8_t sysGetProState();
void sysNotifyParamAltered();

void sysSetProPause();
uint8_t sysProPoll(uint32_t ct);

void sysAddSeconds();
#endif