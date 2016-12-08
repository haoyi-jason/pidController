#ifndef _EEPARAMS_H
#define _EEPRRAMS_H

void WriteToFlash(unsigned long *pArray,unsigned long ulStartAddress, unsigned int uiSize);
void ReadFromFlash(unsigned long *pArray, unsigned long ulStartAddress, unsigned int uiSize);

#endif