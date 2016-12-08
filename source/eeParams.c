#include <ADuCM360.h>
#include <stdio.h>
#include "eeParams.h"
#include "config.h"


void WriteToFlash(unsigned long *pArray,unsigned long ulStartAddress, unsigned int uiSize)
{
   unsigned int uiPollFEESTA = 0;
   volatile unsigned long *flashAddress;
   unsigned int i = 0;
   
   flashAddress = ( unsigned long      *)ulStartAddress;
   FeeWrEn(1);
   uiPollFEESTA = FeeSta();						// Read Status to ensure it is clear
   for (i = 0; i < uiSize; i = i+4)
   { 
      uiPollFEESTA = 0;
      *flashAddress++  = *pArray++;
      do
         {uiPollFEESTA = FeeSta();}
       while((uiPollFEESTA & FEESTA_CMDBUSY) == FEESTA_CMDBUSY);
   }  
   FeeWrEn(0);       // disable a write to Flash memory
}
void ReadFromFlash(unsigned long *pArray, unsigned long ulStartAddress, unsigned int uiSize)
{
   volatile unsigned long *flashAddress;
   unsigned int i = 0;

   flashAddress = ( unsigned long      *)ulStartAddress;
   for (i = 0; i < uiSize; i = i+4)
   {
      *pArray++ = *flashAddress++;
   }
}


