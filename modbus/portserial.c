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
 * File: $Id: portserial.c,v 1.2 2010/06/06 13:46:42 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include <stdlib.h>
#include <aducm360.h>
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define USART0_ENABLED          ( 1 )
#define USART0_IDX              ( 0 )

#define USART1_ENABLED          ( 1 )
#define USART1_IDX              ( USART0_IDX + USART0_ENABLED * 1 )

#define USART_IDX_LAST          ( USART1_IDX )

#define USART_INVALID_PORT      ( 0xFF )
#define USART_NOT_RE_IDX        ( 3 )
#define USART_DE_IDX            ( 4 )

/* ----------------------- Static variables ---------------------------------*/

static unsigned char ucTxBufferEmpty = 0;

static UCHAR    ucUsedPort = USART_INVALID_PORT;

void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
  volatile uint32_t uartIen;
  uartIen = pADI_UART->COMIEN;

    if( xRxEnable )
    {
      uartIen |= COMIEN_ERBFI_EN;
    }
    else
    {
      uartIen &= ~COMIEN_ERBFI_MSK;
    }

    if( xTxEnable )
    {
      uartIen |= COMIEN_ETBEI_EN;
    }
    else
    {
      uartIen &= ~COMIEN_ETBEI_MSK;
    }
    
    pADI_UART->COMIEN = uartIen;
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
  int iBaud,iBits,iFmt;
  BOOL bStatus = FALSE;
 
  switch(eParity){
  case MB_PAR_NONE:
    iFmt = 0;
    bStatus = TRUE;
    break;
  case MB_PAR_ODD:
    iFmt = COMLCR_PEN_EN;
    bStatus = TRUE;
    break;
  case MB_PAR_EVEN:
    iFmt = COMLCR_PEN_EN | COMLCR_EPS_EN; 
    bStatus = TRUE;
    break;
  default:
    bStatus = FALSE;
  }
  
  switch ( ucDataBits )
  {
  case 8:
      iBits = COMLCR_WLS_8BITS;
      bStatus = TRUE;
      break;
  case 7:
      iBits = COMLCR_WLS_7BITS;
      bStatus = TRUE;
      break;
  default:
      bStatus = FALSE;
      bStatus = TRUE;
  }

  if(bStatus == TRUE){
    UrtCfg(pADI_UART,ulBaudRate,iBits,iFmt);
//    UrtIntCfg(pADI_UART,COMIEN_ERBFI | COMIEN_ETBEI | COMIEN_ELSI | COMIEN_EDSSI | COMIEN_EDMAT | COMIEN_EDMAR);
    DioPul(pADI_GP0,0xFF);								              // Enable pullup on P0.7/0.6
    DioCfg(pADI_GP0,0x9000);								              // Configure P0.2/P0.1 for UART
  }
    return bStatus;
}

void
vMBPortSerialClose( void )
{

}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
  //while(ucTxBufferEmpty == 0);
  UrtTx(pADI_UART,ucByte);
  
  return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    *pucByte = UrtRx(pADI_UART);
    return TRUE;
}

void portUartIntHandler()
{
  volatile unsigned char ucCOMSTA0 = 0;
  static volatile unsigned char ucCOMIID0 = 0;
  
  ucCOMSTA0 = UrtLinSta(pADI_UART);
  ucCOMIID0 = UrtIntSta(pADI_UART);
  if((ucCOMIID0 & 0x2) == 0x2){
    //ucTxBufferEmpty = 1;
    pxMBFrameCBTransmitterEmpty();
  }
  if((ucCOMIID0 & 0x4) == 0x4){
    pxMBFrameCBByteReceived();
  }
}
void notifyTxEmpty()
{
    pxMBFrameCBTransmitterEmpty();
  
}
void notifyRxReady()
{
    pxMBFrameCBByteReceived();  
}


