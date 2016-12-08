#include <aducm360.h>
#include "config.h"
#include "keypad.h"
#include "param.h"
#include "userif.h"
#include "haladucm360.h"

#include "sensorif.h"
#include "port.h"
#include "param.h"
#include "keypad.h"
#include "modoperation.h"
#include "mbregs.h"
unsigned ulAbortAddress = 0;
unsigned int uiFEESTA;
volatile unsigned char ucFlashCmdStatus = 0;
volatile unsigned char ucWaitForCmdToComplete = 0;
uint8_t t1_overflow;
uint32_t timerCounter;
static _task_t tasks[] = {
  {0,20,modOpPoll,0},
  {0,10,uiKeyPoll,0},
  {0,20,uiLedPoll,0},
  {0,20,mbTask,0},
};

#define NOF_TASK        4


void taskPoll()
{
  uint8_t sz = sizeof(tasks);
  uint8_t i;
  _task_t *task;
  for(i=0;i<NOF_TASK;i++){
    task = &tasks[i];
    task->cntr++;
    if(task->cntr == task->reload){
      task->cntr = 0;
      if(task->taskFunc){
        task->taskFunc(timerCounter);
      }
    }
    task = task->next;
  }
}

int main()
{
  uint8_t testMode = 0;
  uint32_t counter;
  halInit();
  paramInit();
  keypadInit();
  modOpInit();

  sysInit();

  sysSetMBAddress(0x1);
  sysSetBaudrate(0x2);
  sysSetParity(0x0);
  
  sysSetParam(RRT,10);
  sysSetParam(TMS,10);
  sysSetOpMode(modPRO);
  
  modbusRTUInit(); 
  while(true)
  {
    if(t1_overflow == 1){
      t1_overflow = 0;
      if(testMode) {
        taskPoll();
        continue;
      }
      counter++;
      if((counter % 200) == 0){
        sysAddSeconds();
      }
      taskPoll();
      sysIncCounter();
    }
  }
  return 0;
}


void GP_Tmr1_Int_Handler()
{
  t1_overflow = 1;
  timerCounter+=5;
  GptClrInt(pADI_TM1,TSTA_TMOUT);
}

void ADC1_Int_Handler()
{
  sensorReadADC();
}

void Flsh_Int_Handler()
{
  uiFEESTA = 0;
  uiFEESTA = pADI_FEE->FEESTA;
  
  if((uiFEESTA & 0x30) == 0x00){
    ucFlashCmdStatus = 0;
  }
  if((uiFEESTA & 0x30) == 0x10){
    ucFlashCmdStatus = 1;
  }
  if((uiFEESTA & 0x30) == 0x20){
    ucFlashCmdStatus = 2;
  }
  if((uiFEESTA & 0x30) == 0x30){
    ulAbortAddress = pADI_FEE->FEEADRAH;
    ulAbortAddress = (ulAbortAddress << 16);
    ulAbortAddress |= pADI_FEE->FEEADRAL;
    ucFlashCmdStatus = 3;
  }
  if((uiFEESTA & 0x08) == 0x08){
    
  }
  if((uiFEESTA & 0x4) == 0x04){
    
  }
  
  ucWaitForCmdToComplete = 0;
    
}

void UART_Int_Handler()
{
  //portUartIntHandler();
  
  volatile unsigned char ucCOMSTA0 = 0;
  volatile unsigned char ucCOMIID0 = 0;
  
  ucCOMSTA0 = UrtLinSta(pADI_UART);
  ucCOMIID0 = UrtIntSta(pADI_UART);
  if((ucCOMIID0 & 0x2) == 0x2){
    //ucTxBufferEmpty = 1;
    notifyTxEmpty();
  }
  if((ucCOMIID0 & 0x4) == 0x4){
    //ucComRx = UrtRx(pADI_UART);
    //ucWaitForUart = 0;
    notifyRxReady();
  }
  
}

void PWM0_Int_Handler ()
{
  uint16_t v = sysGetPwmDuty();
  pADI_PWM->PWMCON0 |= (1 << 3);
  pADI_PWM->PWM0COM1 = v;
//  ioGetPwmDuty();
  PwmClrInt(PWMCLRI_PWM0);

}