#ifndef _PARAM_
#define _PARAM_

#define NOF_PARAM       53
#define HEADING 0x55

#define SHOW_STRING     0
#define SHOW_VALUE      1

enum order_e{
  ODR_IDLE,
  ODR_VIEW,
  ODR_SET,
  ODR_CFG,
  ODR_MAN,
  ODR_ATO
};

typedef struct
{
  uint8_t days;
  uint8_t hours;
  uint8_t minutes;
  uint8_t secs;
  uint8_t tmr;
} _datetime_t;

typedef struct param_map_s{
  char disp[3];
  int8_t mbRegOrder;
  int16_t val;
  int8_t digits;
  int16_t lLimit;
  int16_t uLimit;
  int8_t incStep;
  int8_t modified;
}param_map_t;

typedef struct sys_params_s{
  uint8_t flag;
  uint16_t ens;
  uint16_t enp;
  uint16_t lock;
  uint16_t err;
  uint16_t id;
  _datetime_t runDate;
  _datetime_t lastWrite;
  uint16_t atTimeMin;   // auto-tune time, in minutes
}sys_params_t;

void paramInit(void);
void paramSetActiveOrder(uint8_t ord);
void paramMoveAdv();
uint8_t paramGetShowTarget();
void paramSetShowTarget(uint8_t target);
void loadParams();
int16_t readScaledIntParamByIndex(uint16_t index,int16_t *val);
int16_t readIntParamByIndex(uint16_t index,int16_t *val);
int16_t readFloatParamByIndex(uint16_t index,double *val);
int16_t writeScaledIntParamByIndex(uint16_t index,int16_t val);
int16_t writeIntParamByIndex(uint16_t index,int16_t val);
int16_t writeFloatParamByIndex(uint16_t index, double val);
uint8_t paramGetCurrentOrder();

uint8_t paramIsModified(uint8_t index);
void paramClearModifyFlag(uint8_t index);

void writeParams();
#endif