#include <aducm360.h>

void RESET_EXCPT_HANDlr()
{
#ifdef __GNUC__
  unsigned long *pulSrc, *pulDest;
  
  pulSrc = &_etext;
  for(pulDest = &_data;pulDest < &_edata;){
    *pulDst++ = *pulSrc++;
  }
  
  for(pulDest = &_bss;pulDest < &_ebss;){
    *pulDest++ = 0;
  }
#endif
  
  main();
  
  while(1);
}

void SysTick_Handler           () 
{}
void NmiSR                     () 
{}
void FaultISR                  ()
{} 
void MemManage_Handler         () 
{}
void BusFault_Handler          () 
{}
void UsageFault_Handler        ()
{} 
void SVC_Handler               () 
{}
void DebugMon_Handler          () 
{}
void PendSV_Handler            ()
{} 
void WakeUp_Int_Handler()
{
}
void Ext_Int0_Handler ()
{           
  EiClr(EXTINT0); 
}
void Ext_Int1_Handler ()
{           
  EiClr(EXTINT1);
}
void Ext_Int2_Handler ()
{   
  EiClr(EXTINT2);
}
void Ext_Int3_Handler ()
{           
  //keyActive(keyCycle);
  EiClr(EXTINT3);
  
}
void Ext_Int4_Handler ()
{           
  EiClr(EXTINT4);
}
void Ext_Int5_Handler ()
{           
  //keyActive(keyUp);
  EiClr(EXTINT5);
}
void Ext_Int6_Handler ()
{           
  //keyActive(keyDn);
  EiClr(EXTINT6);
}
void Ext_Int7_Handler ()
{           
  //keyActive(keyRm);
  EiClr(EXTINT7);
}
void WDog_Tmr_Int_Handler()
{}
void Test_OSC_Int_Handler()
{}
/*
void GP_Tmr0_Int_Handler()
{
  int portV;
  GptClrInt(pADI_TM0,TSTA_TMOUT);
  //t1_overflow = 1;
}

void GP_Tmr1_Int_Handler()
{
  t1_overflow = 1;
  timerCounter+=5;
  GptClrInt(pADI_TM1,TSTA_TMOUT);
}
void ADC0_Int_Handler()
{}

void ADC1_Int_Handler()
{
  adcIrq();
}
*/
void SINC2_IntHandler()
{
  
}
/*
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
*/
void SPI0_Int_Handler()
{
  
}

void SPI1_Int_Handler()
{
  
}

void I2C0_Slave_Int_Handler()
{
  
}
void I2C0_Master_Int_Handler ()
{
}
void DMA_Err_Int_Handler ()
{
}
void DMA_SPI1_TX_Int_Handler ()
{
}
void DMA_SPI1_RX_Int_Handler ()
{
}
void DMA_UART_TX_Int_Handler ()
{
}

void DMA_I2C0_STX_Int_Handler ()
{
}

void DMA_I2C0_SRX_Int_Handler ()
{
}
void DMA_I2C0_MTX_Int_Handler ()
{
}
void DMA_UART_RX_Int_Handler ()
{
}
void DMA_I2C0_MRX_Int_Handler ()
{
}
void DMA_ADC0_Int_Handler ()
{
}
void DMA_ADC1_Int_Handler ()
{
}
void DMA_DAC_Out_Int_Handler ()
{
}
void DMA_SINC2_Int_Handler ()
{
}
/*
void PWM0_Int_Handler ()
{
  //uint16_t v = ioGetPwmDuty();
  ioGetPwmDuty();
  PwmClrInt(PWMCLRI_PWM0);

}
*/
void PWM1_Int_Handler ()
{
}
void PWM2_Int_Handler ()
{
}
void PWM3_Int_Handler ()
{
}
void PWMTRIP_Int_Handler ()
{
}