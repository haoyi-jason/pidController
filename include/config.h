#ifndef _CONFIG_
#define _CONFIG_

#include <aducm360.h>

/***  definitions */

#define FEE_START_ADDR  0x1f000
#define FEE_HEADING     0xaa
#define FEE_PAGE_SIZE   512
#define FEE_SYS_PARAM_ADDR      0x1f600
#define COUNTRE_PERIOD  5       // ms
#define CT2SEC          200

enum sensor_type_e{
  ST_TC_J,
  ST_TC_K,
  ST_PT100
};


enum param_mbreg_e{
  SYS,
  TPV,
  SP1,
  MV1,
  ALL,
  ALH,
  MOD,
  ADR,
  STA,
  MV2,
  ERR,//10
  DI1,
  VV,
  AA,
  PP,
  KWH,
  ID,
  ENS,
  ENP,
  LED,
  IN,//20
  TSH,
  PLL,
  PLH,
  ALF,
  RRTT,
  CMD,
  PIDP,
  PIDI,
  PIDD,
  O1F,//30
  O1H,
  O1E,
  ALD,
  O2F,
  I1F,
  CIF,
  LOK,
  RRT,
  TMS,
  BAU,//40
  PAR,
  TSP,
  GSP,
  BSP,
  SPL,
  SPH,
  ATU,       // key mode
  DSPC,         // current display mode
  DSPO,         // previous display mode
  PIDM,         // PID mode, {NONE,ATUN}
  MODS,          // MOD saved
  PROS,         // profile run status
};


enum modFunc_e{modNONE=-1,modTCR,modGE1,modGE2,modPRO,modSWM,modMAN,modATO,modTIM};


typedef struct {
  ADI_GPIO_TypeDef *port;
  unsigned long pin;
  unsigned long mode;
}gpio_config_s;

typedef int16_t(*taskProto)(uint32_t);

typedef struct _task_s{
  int cntr;
  int reload;
  taskProto taskFunc;
  struct _task_s *next;
}_task_t;

typedef unsigned char bool;
#define true 1
#define false 0

#endif