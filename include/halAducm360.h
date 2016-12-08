#ifndef _HALADUCM360_
#define _HALADUCM360_

#define NOF_DIG_IN      1
#define NOF_DIG_OUT     3
#define PWM_BASE        1600

void halInit();
void halSetDigOut(unsigned char ch);
void halClrDigOut(unsigned char ch);
void halTglDigOut(unsigned char ch);
uint8_t halReadDigIn(uint8_t ch);
int16_t halReadButtons();
void halSetPwmIoMode(uint8_t mode);
#endif