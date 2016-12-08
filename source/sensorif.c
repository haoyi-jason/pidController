#include <aducm360.h>
#include <math.h>
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


#include "sensorif.h"
#include "config.h"
#include "sysState.h"

#define fVolts (1.2/268435456)

double calcColdJVoltage(double t);
double calcTJTemp(double v);
double calcTKTemp(double v);
double calcRTDTemp(double r);

// Thermocouple lookup tables....
const double C_themocoupleP[THER_N_SEG_P+1] = {0.0, 	15.1417, 	29.8016, 	44.0289, 	57.8675, 	71.3563, 
									84.5295, 	97.4175, 	110.047, 	122.441, 	134.62,		146.602, 
									158.402, 	170.034, 	181.51, 	192.841, 	204.035, 	215.101, 
									226.046, 	236.877, 	247.6,		258.221, 	268.745, 	279.177, 
									289.522, 	299.784, 	309.969, 	320.079, 	330.119, 	340.092, 
									350.001};
const double C_themocoupleN[THER_N_SEG_N+1] = {0.0,		-7.30137,	-14.7101,	-22.2655, 
									-29.9855, 	-37.8791, 	-45.9548, 	-54.2258, 
									-62.7115, 	-71.4378, 	-80.4368,	-89.7453, 
									-99.4048, 	-109.463, 	-119.978, 	-131.025, 
									-142.707, 	-155.173, 	-168.641, 	-183.422, 
									-199.964};
//Cold junction lookup table. Used for converting cold junction temperature to thermocouple voltage
const double C_cold_junctionP[COLDJ_N_SEG_P+1] = {0.0,	0.2435,	0.4899,	0.7393,	0.9920,	1.2479, 1.5072,	1.7698,
												2.0357,	2.3050,	2.5776,	2.8534,	3.1323,	3.4144,	3.6995, 3.9875,
												4.2785,	4.5723,	4.8689,	5.1683,	5.4703};
const double C_cold_junctionN[COLDJ_N_SEG_N+1] = {0.0,		-0.1543,	-0.3072,	-0.4586,	-0.6085,
												-0.7568,	-0.9036,	-1.0489,	-1.1925,	-1.3345,
												-1.4750};
//RTD lookup table
const double C_rtd[] = {-40.0006,-34.6322,-29.2542,-23.8669,-18.4704,-13.0649,-7.65042,-2.22714,3.20489,8.64565,14.0952,
						19.5536,25.0208,30.497,35.9821,41.4762,46.9794,52.4917,58.0131,63.5436,69.0834,74.6325,80.1909,
						85.7587,91.3359,96.9225,102.519,108.124,113.74,119.365,124.999};



volatile long ulADC1DatExt[NOF_SAMPLE];
volatile long ulADC1DatNtc[NOF_SAMPLE];
static uint8_t ucSampleNo;
static uint8_t ucADCInput;
static uint8_t ucTcBreak;
static uint8_t ucSensorType;
static uint8_t ucSensorAcqDone;
double (*calTCFunc)(double v);

static long ulResultExt;
static long ulResultNtc;

void updateADCInput()
{
  switch(ucADCInput){
  case ADC_IN_EXT:
    switch(ucSensorType){
    case ST_TC_J:
      IexcCfg(IEXCCON_PD_off,IEXCCON_REFSEL_Int,IEXCCON_IPSEL1_Off,IEXCCON_IPSEL0_AIN7);
      IexcDat(IEXCDAT_IDAT_200uA,IDAT0Dis);
      AdcPin(pADI_ADC1,ADCCON_ADCCN_AIN1,ADCCON_ADCCP_AIN0);
      AdcRng(pADI_ADC1,ADCCON_ADCREF_INTREF,ADCMDE_PGA_G32,ADCCON_ADCCODE_INT);
      calTCFunc = calcTJTemp;
      break;
    case ST_TC_K:
      IexcCfg(IEXCCON_PD_off,IEXCCON_REFSEL_Int,IEXCCON_IPSEL1_Off,IEXCCON_IPSEL0_AIN7);
      IexcDat(IEXCDAT_IDAT_200uA,IDAT0Dis);
      calTCFunc = calcTKTemp;
      AdcPin(pADI_ADC1,ADCCON_ADCCN_AIN1,ADCCON_ADCCP_AIN0);
      AdcRng(pADI_ADC1,ADCCON_ADCREF_INTREF,ADCMDE_PGA_G32,ADCCON_ADCCODE_INT);
      break;
    case ST_PT100:
      IexcCfg(IEXCCON_PD_off,IEXCCON_REFSEL_Int,IEXCCON_IPSEL1_AIN5,IEXCCON_IPSEL0_AIN6);
      IexcDat(IEXCDAT_IDAT_100uA,IDAT0Dis);
      calTCFunc = calcRTDTemp;
      AdcPin(pADI_ADC1,ADCCON_ADCCN_AIN1,ADCCON_ADCCP_AIN0);
      AdcRng(pADI_ADC1,ADCCON_ADCREF_EXTREF,ADCMDE_PGA_G32,ADCCON_ADCCODE_INT);
      break;      
    }
    break;
  case ADC_IN_NTC:
    AdcPin(pADI_ADC1,ADCCON_ADCCN_AGND,ADCCON_ADCCP_AIN11);
    AdcRng(pADI_ADC1,ADCCON_ADCREF_INTREF,ADCMDE_PGA_G1,ADCCON_ADCCODE_INT);
    IexcDat(IEXCDAT_IDAT_0uA ,IDAT0Dis);
    break;
  }
}

void sensorReadADC()
{
  volatile unsigned int uiADCSTA = 0;
  volatile long ulADC1DAT = 0;
  uint8_t i;
  uiADCSTA = AdcSta(pADI_ADC1);
  
  ulADC1DAT = AdcRd(pADI_ADC1);
  if(ucSampleNo < NOF_SAMPLE){
    switch(ucADCInput){
    case ADC_IN_EXT:
      ulADC1DatExt[ucSampleNo++] = ulADC1DAT;
      if((uiADCSTA & 0x10) == 0x10){
        ucTcBreak = 1;
      }else{
        ucTcBreak = 0;
      }
      break;
    case ADC_IN_NTC:
      ulADC1DatNtc[ucSampleNo++] = ulADC1DAT;
      break;
    }
  }else{
    ucSampleNo = 0;
    switch(ucADCInput){
    case ADC_IN_EXT:
      ucADCInput = ADC_IN_NTC;
      ulResultExt = 0;
      for(i=0;i<NOF_SAMPLE;i++){
        ulResultExt += ulADC1DatExt[i];
      }
      break;
    case ADC_IN_NTC:
      ucADCInput = ADC_IN_EXT;
      ulResultNtc = 0;
      for(i=0;i<NOF_SAMPLE;i++){
        ulResultNtc += ulADC1DatNtc[i];
      }
      ucSensorAcqDone = 1;
      break;
    }
    updateADCInput();
  }
  
}

void sensorInit()
{
  ucADCInput = ADC_IN_EXT;
  ucSampleNo = 0;
  ucSensorAcqDone = 0;
  updateADCInput();
  ucSensorType = sysGetSensorType();
}

int16_t sensorAcqDone()
{
  return ucSensorAcqDone;
}

void resetSensorDone()
{
  ucSensorAcqDone = 0;
}

uint8_t sensorGetTemp(double *coldJ, double *sensortemp)
{
  double fVExt = 0;
  double fVNtc = 0;
  double fRNtc;
  double fTNtc;
  double fRRtd;
  fVNtc = (double)(ulResultNtc*fVolts);
  fVNtc /= NOF_SAMPLE;
  
  // calculate NTC temperature
  fRNtc = 3000*(3.3/fVNtc - 1.0);
  // NTD: 10K, 3380
  fTNtc = fRNtc/10000;
  fTNtc = log(fTNtc);
  fTNtc /= 3380;
  fTNtc += (1.0/(273.15+25));
  fTNtc = 1.0/fTNtc;
  fTNtc -= 273.15;
  *coldJ = fTNtc;
  
  switch(ucSensorType){
  case ST_TC_J:
    fVExt = (double)(ulResultExt*fVolts);
    fVExt /= NOF_SAMPLE;
    *sensortemp = calcTJTemp(fVExt);
    break;
  case ST_TC_K:
    fVExt = (double)(ulResultExt*fVolts);
    fVExt /= NOF_SAMPLE;
    *sensortemp = calcTKTemp(fVExt);
    break;
  case ST_PT100:
    fVExt = (double)(ulResultExt/268435456);
    fRRtd = fVExt * 11800;
    *sensortemp = calcRTDTemp(fRRtd);
    break;
  }
  
  ucSensorType = sysGetSensorType();
  return ucTcBreak;
}


double calcRTDTemp(double r)
{
	double t;
	int j;
	j=(double)(r-RMIN)/RSEG;       // determine which coefficients to use
	if (j<0)               // if input is under-range..
		j=0;                // ..then use lowest coefficients
	else if (j>NSEG-1)     // if input is over-range..
		j=NSEG-1;            // ..then use highest coefficients
	t = C_rtd[j]+(r-(RMIN+RSEG*j))*(C_rtd[j+1]-C_rtd[j])/RSEG;
	return t;
}

double calcTKTemp(double v)
{
	double fresult = 0;
	signed int j = 0;
	double fMVthermocouple = v*1000;			//thermocouple voltage in mV
	if (fMVthermocouple >= 0)
	{
  		j=(fMVthermocouple - THER_V_MIN_P) / THER_V_SEG_P;			// determine which coefficient to use
  		if (j>THER_N_SEG_P-1)     									// if input is over-range..
    		j=THER_N_SEG_P-1;            							// ..then use highest coefficients
		
		// Use the closest known temperature value and then use a linear approximation beetween the
		// enclosing data points
  		fresult = C_themocoupleP[j] + (fMVthermocouple - (THER_V_MIN_P+THER_V_SEG_P*j) )*
			(C_themocoupleP[j+1]-C_themocoupleP[j]) / THER_V_SEG_P;		 // Slope
	}
	else if (fMVthermocouple < 0)
	{
		j=(fMVthermocouple - THER_V_MIN_N) / THER_V_SEG_N;		// determine which coefficient to use
		if (j>THER_N_SEG_N-1)    								// if input is over-range..
    		j=THER_N_SEG_N-1;          							// ..then use highest coefficients
		fresult = C_themocoupleN[j] + (fMVthermocouple- (THER_V_MIN_N+THER_V_SEG_N*j) )*
			(C_themocoupleN[j+1]-C_themocoupleN[j]) / THER_V_SEG_N;
	} 
   	return  fresult; 
}

double calcTJTemp(double v)
{
  	double fresult = 0;
	signed int j = 0;
	double fMVthermocouple = v*1000;			//thermocouple voltage in mV
	if (fMVthermocouple >= 0)
	{
  		j=(fMVthermocouple - THER_V_MIN_P) / THER_V_SEG_P;			// determine which coefficient to use
  		if (j>THER_N_SEG_P-1)     									// if input is over-range..
    		j=THER_N_SEG_P-1;            							// ..then use highest coefficients
		
		// Use the closest known temperature value and then use a linear approximation beetween the
		// enclosing data points
  		fresult = C_themocoupleP[j] + (fMVthermocouple - (THER_V_MIN_P+THER_V_SEG_P*j) )*
			(C_themocoupleP[j+1]-C_themocoupleP[j]) / THER_V_SEG_P;		 // Slope
	}
	else if (fMVthermocouple < 0)
	{
		j=(fMVthermocouple - THER_V_MIN_N) / THER_V_SEG_N;		// determine which coefficient to use
		if (j>THER_N_SEG_N-1)    								// if input is over-range..
    		j=THER_N_SEG_N-1;          							// ..then use highest coefficients
		fresult = C_themocoupleN[j] + (fMVthermocouple- (THER_V_MIN_N+THER_V_SEG_N*j) )*
			(C_themocoupleN[j+1]-C_themocoupleN[j]) / THER_V_SEG_N;
	} 
   	return  fresult;
}

// Convert RTD temperature to its thermocouple equivalent voltage
double calcColdJVoltage(double t)
{
	double fresult = 0;
	signed int j = 0;
	if (t >= 0)
	{
  		j=(t - COLDJ_T_MIN_P) / COLDJ_T_SEG_P;				// determine which coefficient to use
  		if (j>COLDJ_N_SEG_P-1)     							// if input is over-range..
    		j=COLDJ_N_SEG_P-1;            					// ..then use highest coefficients
		
		// Use the closest known voltage and then use a linear approximation beetween the
		// enclosing data points
  		fresult = C_cold_junctionP[j] + (t - (COLDJ_T_MIN_P+COLDJ_T_SEG_P*j) )*
			(C_cold_junctionP[j+1]-C_cold_junctionP[j]) / COLDJ_T_SEG_P;		 // Slope
	
	}
	else if (t < 0)
	{
		j=(j - COLDJ_T_MIN_N) / COLDJ_T_SEG_N;				// determine which coefficient to use
		if (j>COLDJ_N_SEG_N-1)    							// if input is over-range..
    		j=COLDJ_N_SEG_N-1;         						// ..then use highest coefficients

		fresult = C_cold_junctionN[j] + (t - (COLDJ_T_MIN_N+COLDJ_T_SEG_N*j) )*
			(C_cold_junctionN[j+1]-C_cold_junctionN[j]) / COLDJ_T_SEG_N;
	}
	return fresult/1000.0;
}

