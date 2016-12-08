#include <stdio.h>
#include <string.h>
#include <aducm360.h>

#include "clklib.h"
#include "intlib.h"
#include "iexclib.h"
#include "diolib.h"
#include "wdtlib.h"
#include "wutlib.h"
#include "urtlib.h"
#include "gptlib.h"
#include "adclib.h"
#include "daclib.h"
#include "rstlib.h"
#include "pwmlib.h"
#include "i2clib.h"
#include "dmalib.h"

#include "config.h"
#include "haladucm360.h"

gpio_config_s digIn[] = {
  {pADI_GP1,(1<<1),0},
};

gpio_config_s Buttons[] = {
  {pADI_GP1,(1<<0),0},
  {pADI_GP1,(1<<5),0},
  {pADI_GP1,(1<<6),0},
  {pADI_GP1,(1<<7),0},
};

gpio_config_s digOut[] = {
  {pADI_GP1,4,0},       // relay control
  {pADI_GP0,4,0},       // rs485_switch
  {pADI_GP1,3,0},
};


void clockInit()
{
  WdtCfg(T3CON_PRE_DIV1,T3CON_IRQ_EN,T3CON_PD_DIS);
  ClkDis(0);
  ClkCfg(CLK_CD0,CLK_HF,CLKSYSDIV_DIV2EN_DIS,CLK_UCLKCG);
  ClkSel(CLK_CD0,CLK_CD0,CLK_CD0,CLK_CD0);
}
void i2cInit()
{
  uint8_t uxI2CRxData[16];
  // gipio for I2C bus, P2.0 & P2.1
  pADI_GP2->GPCON &= 0xfff0;
  pADI_GP2->GPCON |= 0x0005;
  
  // start I2C Bus
  I2cBaud(0x4f,0x4e);
  I2cMCfg(I2CMCON_TXDMA|I2CMCON_RXDMA,I2CMCON_IENCMP_EN|
    I2CMCON_IENNACK_EN|I2CMCON_IENALOST_EN|
    I2CMCON_IENTX_EN|I2CMCON_IENRX_EN,I2CMCON_MAS_EN);    // Enable I2C master mode, Enable DMA on tx and Rx
  
  // Setup I2C Master Tx DMA structure
  DmaPeripheralStructSetup(I2CMTX_C,DMA_DSTINC_NO|
     DMA_SRCINC_BYTE|DMA_SIZE_BYTE);                      // Setup I2C Master Tx DMA structure
  // Setup I2C Master Rx DMA structure
  DmaPeripheralStructSetup(I2CMRX_C,DMA_DSTINC_BYTE|      // Setup I2C master Rx DMA structure - basic transfers
    DMA_SRCINC_NO|DMA_SIZE_BYTE);
  DmaStructPtrInSetup(I2CMRX_C,16,uxI2CRxData);           // Setup i2C master Rx destination
  DmaCycleCntCtrl(I2CMRX_C,16,DMA_DSTINC_BYTE|
     DMA_SRCINC_NO|DMA_SIZE_BYTE|DMA_BASIC);
  DmaClr(DMARMSKCLR_I2CMRX,0,0,0);                        // Disable masking of I2C master rx channel
  DmaSet(0,DMAENSET_I2CMRX,0,0);                          // Enable i2C master Rx DMA channel

}
void gpioInit()
{
  // gpio config P0.0~P0.3 as GPIO_OUTPUT
  pADI_GP0->GPCON &= 0xff00;
  pADI_GP0->GPCON |= 0x0;
  pADI_GP0->GPOEN |= 0x0f;
  pADI_GP0->GPCLR |= 0x0f;      // dimming
  
  static int i;
  uint32_t tmp;
  /* configurateio i/o of
    1. RS-485 Gate control (P0.4, OUT)
    2. Isolated digital IN (P1.1, IN, falling trigger)
    3. digital Out 0 (P1.3, OUT)
    4. digital Out 1 (P1.4)
    4. PWM Triac control (P1.2, OUT, PWM)
    4. Buttons, (P1.0,1.5~1.7, IN)
  */
  
  // output pins
  tmp = pADI_GP0->GPOEN;
  tmp |= (1 << 4);
  pADI_GP0->GPOEN = tmp;
  
  tmp = pADI_GP1->GPOEN;
  tmp |= (1<<4);
  tmp |= (1<<3);
  pADI_GP1->GPOEN = tmp;

  // pwm pin
  DioCfg(pADI_GP1,0x10);        // P1.2, PWM0
  PwmInit(UCLK_8,PWMCON0_PWMIEN_EN,PWMCON0_SYNC_DIS,PWMCON1_TRIPEN_DIS);
  
  
    // input pins
  DioPul(pADI_GP1,(1<<0) | (1<<1) | (1<<5) | (1<<6) | (1<<7));

  PwmLoad(PWMCON0_LCOMP_EN);
  PwmTime(PWM0_1,PWM_BASE,PWM_BASE>>2,0);
  halSetPwmIoMode(0);
  PwmGo(PWMCON0_ENABLE_EN,PWMCON0_MOD_DIS);
  NVIC_EnableIRQ(PWM_PAIR0_IRQn);
  
}

void nvicInit()
{

  NVIC_EnableIRQ(DMA_I2CM_TX_IRQn);                      // I2C Master Tx DMA interrupt enable
  NVIC_EnableIRQ(DMA_I2CM_RX_IRQn);                      // I2C Master Rx DMA interrupt enable
  NVIC_EnableIRQ(FLASH_IRQn);
  NVIC_EnableIRQ(UART_IRQn);
  NVIC_EnableIRQ(ADC1_IRQn);
  //NVIC_EnableIRQ(TIMER0_IRQn);
  NVIC_EnableIRQ(TIMER1_IRQn);
}

void dmaInit()
{
  DmaBase();
}

void timerInit()
{
  // initial timer 0 for 1ms resolution
  // clock source: uclk = 16MHz Ext. crystal
  // pre-scalar: 256, time-base = 16M/256 = 62500
  // Reload value = 625 = 10ms refresh rate
  //GptLd(pADI_TM0,625); 
  //GptCfg(pADI_TM0,TCON_CLK_UCLK,TCON_PRE_DIV256,TCON_MOD|TCON_RLD|TCON_ENABLE);  // T0 config, Uclk/256, 
  
  // initial timer 1 for 5 ms
  GptLd(pADI_TM1,313);
  GptCfg(pADI_TM1,TCON_CLK_UCLK,TCON_PRE_DIV256,TCON_MOD | TCON_RLD | TCON_ENABLE);

}

void adcInit()
{
  // initial ADC1
  AdcGo(pADI_ADC1,ADCMDE_ADCMD_IDLE);
  AdcFlt(pADI_ADC1,40,1,FLT_NORMAL | ADCFLT_NOTCH2 | ADCFLT_CHOP | ADCFLT_RAVG2);
  AdcRng(pADI_ADC1,ADCCON_ADCREF_INTREF,ADCMDE_PGA_G32,ADCCON_ADCCODE_INT);
  AdcBuf(pADI_ADC1,ADCCFG_EXTBUF_VREFPN,ADC_BUF_ON);
  AdcPin(pADI_ADC1,ADCCON_ADCCN_AIN1,ADCCON_ADCCP_AIN0);
  AdcMski(pADI_ADC1,ADCMSKI_RDY,1);
  AdcGo(pADI_ADC1,ADCMDE_ADCMD_CONT);
  
}

void halInit()
{
  clockInit();
  gpioInit();
  dmaInit();
  i2cInit();
  adcInit();
  timerInit();
  nvicInit();
  
}

void halSetDigOut(unsigned char ch)
{
  if(ch < NOF_DIG_OUT){
    digOut[ch].port->GPSET = (1 << digOut[ch].pin);
  }
}
void halClrDigOut(unsigned char ch)
{
  if(ch < NOF_DIG_OUT){
    digOut[ch].port->GPCLR = (1 << digOut[ch].pin);
  }
}
void halTglDigOut(unsigned char ch)
{
  if(ch < NOF_DIG_OUT){
    digOut[ch].port->GPTGL = (1 << digOut[ch].pin);
  }
}

uint8_t halReadDigIn(uint8_t ch)
{
  uint8_t ret = 0x00;
  uint8_t i;
  int di;
  
  di = DioRd(digIn[ch].port);
  if((di & digIn[ch].pin) == digIn[ch].pin) return 1;
  else return 0;
}

int16_t halReadButtons()
{
  int16_t ret = 0x0;
  uint8_t i;
  int rd;
  rd = DioRd(Buttons[i].port);
  for(i = 0; i< 4; i++){
    if((rd & Buttons[i].pin) == Buttons[i].pin){
      ret |= (1 << i);
    }
  }
  return ret;  
}

/*
void halPwmUpdate()
{
  PwmLoad(PWMCON0_LCOMP_DIS);
  PwmTime(PWM0_1,PWM_BASE,outDuty,0);
  PwmLoad(PWMCON0_LCOMP_EN);
}
*/
void halSetPwmIoMode(uint8_t mode)
{
  uint32_t cfg;
  cfg = pADI_GP1->GPCON;
  if(mode == 0){        // GPIO
      //NVIC_DisableIRQ(PWM_PAIR0_IRQn);
    pADI_GP1->GPCON &= ~0x30;
    pADI_GP1->GPOEN |= (1 << 2);
    pADI_GP1->GPOUT &= ~(1<<2);
    /*
      DioCfg(pADI_GP1,0x00);  
      DioOen(pADI_GP1,(1<<2));
      DioClr(pADI_GP1,(1<<2));
      */  
  }
  else{
    pADI_GP1->GPCON |= 0x10;
      //DioCfg(pADI_GP1,0x10);        // P1.2, PWM0
//      NVIC_EnableIRQ(PWM_PAIR0_IRQn);

  }
}




