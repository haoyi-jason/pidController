#include <aducm360.h>
#include <stdio.h>

#include "mb.h"
#include "mbregs.h"
#include "sysState.h"
#include "config.h"
#include "param.h"
//#include ".h"
//#include "controlloop.h"

mb_holding_reg_t *mbRegRoot;
static uint8_t addrSet = 0;
static uint8_t mbShallReset = 0;

void mbReset()
{
  mbShallReset = 1;
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{

  
    return MB_ENOREG;
}

uint16_t byteSwap16(uint16_t v)
{
  uint16_t vret;
  vret = ((v&0xff)<<8) | (v >> 8);

  return vret;
}

int16_t endienSwap(char *dest,char *src, uint8_t len)
{
  uint8_t i;
  dest += len;
  for(i=0;i<len;i++){
    *(dest--) = *(src++);
  }
  
  return 0;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
  USHORT usAddr = usAddress;
  USHORT nofReg = usNRegs;
  USHORT i;
  USHORT regVal;
  ULONG  regValL;
  //key_map_t *key;
  if(eMode == MB_REG_READ){
    for(i=0;i<nofReg;i++){
      if(usAddr == 99){
        regVal = sysGetMBAddress();
      }
      else{
        regVal = sysGetParam(usAddr);
        regVal = byteSwap16(regVal);  
          memcpy(pucRegBuffer,&regVal,2);
          usAddr++;
          pucRegBuffer += 2;
       // }
        //regVal = readKeyByReg(usAddr);
      }
    }
  }
  else{ // write process
    for(i=0;i<usNRegs;i++){
      memcpy(&regVal,pucRegBuffer,2);
      regVal = byteSwap16(regVal);  
      //sysSetParam(usAddr,regVal);
      
      int16_t keyVal;
      switch(usAddr){

      case CIF:
        // check if MOD == modPRO
        //readIntParamByIndex(MOD,&keyVal);
        keyVal = sysGetOpMode();
        if(keyVal == modPRO){
          switch(regVal){
          case 1: //
            sysSetProStart();
            break;
          case 2:
            sysSetProStop();
            break;
          case 3:
            break;
          }
        }
        break;
        /*
      case IN:
        sysSetSensorType(regVal);
        break;
        */
      case 99:
        sysSetMBAddress(regVal);
        break;
      default:
        sysSetParam(usAddr,regVal);
      }
      pucRegBuffer += 2;
      usAddr++;
    }
  }
  return MB_ENOERR;
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}

void writeMBHoldingReg(uint16_t addr, uint16_t dat)
{
  uint16_t writeAdr = addr;
  mbRegWrite(addr,dat);
}

void setAddrState(uint8_t state)
{
  addrSet = state;
}

void modbusRTUInit(void)
{
  static uint8_t mbSla;
  uint16_t val;
  static uint8_t mbPar;
  static uint32_t mbBau;
  eMBErrorCode eStatus;
  eMBClose();
//  eMBDisable();
  //mbSla = readKeyByReg(ADR);
  mbSla = sysGetMBAddress();
//  readIntParamByIndex(ADR,(int16_t*)&mbSla);
//  readIntParamByIndex(PAR,&val);
  //val = readKeyByReg(PAR);
  switch(sysGetParity()){
  case 0:mbPar = MB_PAR_EVEN;break;
  case 1:mbPar = MB_PAR_ODD;break;
  case 2:mbPar = MB_PAR_NONE;break;
  }
//  readIntParamByIndex(BAU,&val);
  switch(sysGetBaudrate()){
  case 0: mbBau = 9600;break;
  case 1: mbBau = 19200;break;
  case 2: mbBau = 38400;break;
  case 3: mbBau = 115200;break;
  }

  eStatus = eMBInit(MB_RTU,mbSla,0,mbBau,mbPar);
  eStatus = eMBEnable();
  
}

int16_t mbTask(uint32_t time)
{
  if(mbShallReset == 1){
    mbShallReset = 0;
    modbusRTUInit();
  }else{
    eMBPoll();
  }
}   