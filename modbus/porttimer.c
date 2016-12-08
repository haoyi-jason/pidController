/*
 * FreeModbus Libary: Atmel AT91SAM3S Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: porttimer.c,v 1.2 2010/06/06 13:46:42 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/

/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"
#include "mb.h"
#include "mbport.h"

USHORT usTimeOut;

/* ----------------------- Defines ------------------------------------------*/


/* ----------------------- Static variables ---------------------------------*/
static USHORT usTimeOut;
/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
  int ld;
  // set timer 0 to re-start every 50us
  // interval = LD* Divider/16MHz
  // LD = interval*16M/Divider = 50(us)*16M/1 = 800 
  ld = usTim1Timerout50us * 800;
  NVIC_DisableIRQ(TIMER0_IRQn);
  GptLd(pADI_TM0,ld); 
  //GptCfg(pADI_TM0,TCON_CLK_UCLK,TCON_PRE_DIV1,TCON_MOD|TCON_RLD|TCON_ENABLE); 
  
  //NVIC_EnableIRQ(TIMER0_IRQn);
  
  return TRUE;
}

void 
vMBPortTimerClose( void )
{
    //NVIC_DisableIRQ( TCXIRQ );
    //PMC_DisablePeripheral( ID_TC0 );
  NVIC_DisableIRQ(TIMER0_IRQn);
}

void
vMBPortTimersEnable(  )
{
  //pADI_TM0->VAL = 0;
  usTimeOut = 0;
  pADI_TM0->VAL = 0;
  GptCfg(pADI_TM0,TCON_CLK_UCLK,TCON_PRE_DIV1,TCON_MOD|TCON_RLD|TCON_ENABLE); 
  NVIC_EnableIRQ(TIMER0_IRQn);
  
  /*
#if MB_TIMER_DEBUG == 1
    PIO_Set( &xTimerDebugPins[0] );  
#endif  
    TCX->TC_CHANNEL[TCCHANNEL].TC_IER = TC_IERX_CPAS;
    TC_Start( TCX, 0 );
*/
}

void
vMBPortTimersDisable(  )
{
  GptCfg(pADI_TM0,TCON_CLK_UCLK,TCON_PRE_DIV1,TCON_MOD|TCON_RLD|TCON_ENABLE_DIS); 
  NVIC_DisableIRQ(TIMER0_IRQn);
//    TC_Stop( TCX, 0 );
#if MB_TIMER_DEBUG == 1
//    PIO_Clear( &xTimerDebugPins[0] );
#endif   
}

void
vMBPortTimersDelay( USHORT usTimeOutMS )
{
  
  usTimeOut = usTimeOutMS*20;
  while(usTimeOut != 0){}
}

void

GP_Tmr0_Int_Handler( void )
{
  GptClrInt(pADI_TM1,TSTA_TMOUT);
  if(usTimeOut > 0) usTimeOut --;
  ( void )pxMBPortCBTimerExpired(  );
}

