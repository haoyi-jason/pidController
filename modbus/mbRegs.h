#ifndef _MBREGS_H
#define _MBREGS_H

typedef struct mb_holding_reg_s{
  uint8_t addr;
  uint16_t val;
  struct mb_holding_reg_s *next;
}mb_holding_reg_t;

void mbRegInit();
mb_holding_reg_t *mbRegAddIndex(uint16_t addr);
uint16_t mbRegRead(uint16_t addr);
uint8_t mbRegWrite(uint16_t addr,uint16_t val);
void setAddrState(uint8_t state);

void modbusRTUInit(void);
int16_t mbTask(uint32_t time);
void mbReset();
#endif