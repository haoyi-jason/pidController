#include "pid.h"
#include "config.h"
#include "sysState.h"

double dispKp;
double dispKi;
double dispKd;
static double kp;
static double ki;
static double kd;
int controllerDirection;
double *myInput;
double *myOutput;
double *mySetpoint;

double theInput,theOutput,theSetpoint;

unsigned long lastTime;
double Iterm,lastInput;
unsigned long sampleTime;
double outMin,outMax;
bool inAuto;

unsigned long pidTimer;
//double Input,Output,SetPoint;
double KP,KI,KD;
bool bAtuneState;
// variables for ATUne
bool isMax,isMin;
double *atIn,*atOut;
//double setPoint;
double noiseBand;
int controlType;
bool running;
uint32_t peak1,peak2;
int nLookBack;
int peakType;
double lastInputs[101];
double peaks[10];
int peakCount;
bool justchanged;
bool justevaled;
double absMax,absMin;
double oStep;
double outputStart;
double Ku,Pu;

void pidSetTmrUpdate()
{
  pidTimer++;
}

unsigned long millis()
{
  uint32_t retv = sysGetTimer();
  return retv*5;
}

void pidInit(double *input,
             double *output,
             double *setPoint,
             double Kp,
             double Ki,
             double Kd,
             int controlDir)
{
  myOutput = output;
  myInput = input;
  mySetpoint = setPoint;
  inAuto = false;
  sampleTime = 100;
  pidSetOutputLimits(0,100);
  
  pidSetControllerDirection(controlDir);
  pidSetTunings(Kp,Ki,Kd);
  
  lastTime = millis();
}
/*
void pidInit2(double Kp, double Ki,double Kd, int controlDir)
{
  inAuto = false;
  myOutput = &theOutput;
  myInput = &theInput;
  mySetpoint = &theSetpoint;
  sampleTime = 100;     // 100 ms
  pidSetOutputLimits(0,100);
  pidSetControllerDirection(controlDir);
  pidSetTunings(kp,ki,kd);
  lastTime = millis();
}
*/

bool pidCompute()
{
  double input;
  double error;
  double dInput;
  double output;
  static unsigned long timeChange;
  if(!inAuto) return false;
  
  unsigned long now = millis();
//  unsigned long now;
  timeChange = now - lastTime;
  
  if(timeChange >= sampleTime)
  {
 
    input = *myInput;
      error = *mySetpoint - input;
      Iterm += (ki * error);
      
      if(Iterm > outMax) Iterm = outMax;
      else if(Iterm < outMin) Iterm = outMin;
      
      dInput = (input - lastInput);
      
      output = kp*error +Iterm - kd*dInput;
      
      if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
      
      *myOutput = output;
      
      lastInput = input;
      lastTime = now;
       
      return true;
  }
  else return false;
}

double pidGetOutput()
{
  return *myOutput;
}

void pidSetTunings(double Kp,double Ki,double Kd)
{
  double sampleTimeInSec = ((double)sampleTime)/1000;
  if(Kp < 0 || Ki < 0 || Kd < 0) return;

   kp = Kp;
   ki = Ki * sampleTimeInSec;
   kd = Kd / sampleTimeInSec;
   
   if(controllerDirection == REVERSE){
     kp = 0-kp;
     ki = 0-ki;
     kd = 0-kd;
   }
}

void pidSetSampleTime(int newSampleTime)
{
  double ratio;
  if(newSampleTime > 0){
    ratio = (double)newSampleTime / (double)sampleTime;
    
    ki *= ratio;
    kp /= ratio;
    sampleTime = (unsigned long)newSampleTime;
  }
}

void pidSetOutputLimits(double min,double max)
{
  if(min > max) return;
  outMin = min;
  outMax = max;
  
  if(inAuto){
    if(*myOutput > outMax) *myOutput = outMax;
    else if(*myOutput < outMin) *myOutput = outMin;
    
    if(Iterm > outMax) Iterm = outMax;
    else if(Iterm < outMin) Iterm = outMin;
  }
}

void pidSetMode(int mode)
{
  bool newAuto = (mode == AUTOMATIC);
  if(newAuto == !inAuto){
    initialize();
  }
  inAuto = newAuto;
}

void initialize()
{
  Iterm = *myOutput;
  lastInput = *myInput;
  if(Iterm > outMax) Iterm = outMax;
  else if(Iterm < outMin) Iterm = outMin;
}

void pidSetControllerDirection(int dir)
{
  if(inAuto && dir != controllerDirection){
    kp = (0-kp);
    ki = (0-ki);
    kd = (0-kd);
  }
  controllerDirection = dir;
}

double pidGetKp()
{
  return dispKp;
}

double pidGetKi()
{
  return dispKi;
}

double pidGetKd()
{
  return dispKd;
}

int pidGetMode()
{
  return inAuto?AUTOMATIC:MANUAL;
}

int pidGetDirection()
{
  return controllerDirection;
}

void pidSetPID(double p,double i,double d)
{
  KP = p; KI = i; KD = d;
}

void pidSetPoint(double v)
{
  *mySetpoint = v;
}

void pidSetInput(double v)
{
  *myInput = v;
}

/*
void pidProcessInit()
{
  pidTimer = 0;
  pidInit(&Input,&Output,&SetPoint,KP,KI,KD,DIRECT);
  pidSetMode(AUTOMATIC);
}

void pidProcess(double *in, double *out)
{
  Input = *in;
  pidCompute();
  *out = Output;
}
*/
float pidProcess2(double in)
{
  *myInput = in;
  pidCompute();
  return (theOutput);
}

void pidATuneInit(double *in, double *out)
{
  atIn = in;
  atOut = out;
  controlType = 0;
  noiseBand = 1.0;
  running = false;
  oStep = 50;
  pidATSetLoopbackSec(30);
  lastTime = millis();  
  bAtuneState = false;
}

void pidATuneSetState(bool s)
{
  bAtuneState = s;
}

bool pidATuneGetState()
{
  return bAtuneState;
}

void pidATCancel()
{
  running = false;
}

int pidATRuntime()
{
  int i;
  double val;
  static double avgSeparation;
  uint32_t now = millis();
  double refVal;
  justevaled = false;
  if(peakCount > 9 && running){
    running = false;
    pidATFinishUp();
    return 1;
  }
  
  if((now - lastTime) < sampleTime) 
    return 0;
  lastTime = now;
  refVal = *atIn;
  justevaled = true;
  if(!running){
    running = true;
    peakType = 0;
    peakCount = 0;
    justchanged = false;
    absMax = refVal;
    absMin = refVal;
    //setPoint = refVal;
    outputStart = *atOut;
    // remove on 9/14, this value will set later
    //*atOut = outputStart + oStep;
  }
  else{
    if(refVal > absMax) absMax = refVal;
    if(refVal < absMin) absMin = refVal;
  }
  
//  if(refVal > *mySetpoint+noiseBand) *atOut = outputStart - oStep;
//  else if(refVal < *mySetpoint - noiseBand) *atOut = outputStart + oStep;
  // in AT mode, only 100% & 0% will drive the heater,
  // change values to 100 & 0 accordingly
  if(refVal > *mySetpoint+noiseBand) *atOut = 0;
  else if(refVal < *mySetpoint - noiseBand) *atOut = 100;
  
  isMax = true;
  isMin = true;
  
  for(i = nLookBack -1;i>=0;i--){
    val = lastInputs[i];
    if(isMax) isMax = refVal>val;
    if(isMin) isMin = refVal < val;
    lastInputs[i+1] = lastInputs[i];
  }
  
  lastInputs[0] = refVal;
  if(nLookBack < 9) return 0;
  
  if(isMax){
    if(peakType == 0) peakType = 1;
    if(peakType == -1){
      peakType = 1;
      justchanged = true;
      peak2 = peak1;
    }
    peak1 = now;
    peaks[peakCount] = refVal;
  }
  else if(isMin){
    if(peakType == 0) peakType = -1;
    if(peakType == 1){
      peakType = -1;
      peakCount++;
      justchanged = true;
    }
    if(peakCount < 10) peaks[peakCount] = refVal;
  }
  
  if(justchanged && peakCount > 2){
    avgSeparation = (abs(peaks[peakCount-1] - peaks[peakCount-2])+
                     abs(peaks[peakCount-2] - peaks[peakCount-3]))/2;
    if(avgSeparation < 0.05*(absMax - absMin)){
//    if(avgSeparation < 0.1*(absMax - absMin)){
      pidATFinishUp();
      running = false;
      return 1;
    }
  }
  justchanged = false;
  return 0;
  
    
}

double pitATGetOutput()
{
  return *atOut;
}

void pidATFinishUp()
{
  *atOut = outputStart;
  Ku = 4*(2*oStep)/((absMax-absMin)*3.14159);
  Pu = (double)(peak1-peak2)/1000;
}

double pidATGetKp()
{
  return controlType == 1?0.6*Ku:0.4*Ku;
}

double pidATGetKi()
{
  return controlType == 1?1.2*Ku/Pu:0.48*Ku/Pu;
}  

double pidATGetKd()
{
  return controlType == 1?0.075*Ku*Pu:0;
}  

void pidATSetOutputStep(double step)
{
  oStep = step;
}

double pidAtGetOutputStep()
{
  return oStep;
}

void pidATSetControlType(int type)
{
  controlType = type;
}

int pidATGetControlType()
{
  return controlType;
}

void pidATSetNoiseBand(double band)
{
  noiseBand = band;
}

double pidATGetNoiseBand()
{
  return noiseBand;
}

void pidATSetLoopbackSec(int val)
{
  if(val < 1) val = 1;
  if(val < 25){
    nLookBack = val * 4;
    sampleTime = 250;
  }
  else{
    nLookBack = 100;
    sampleTime = val*10;
  }
}
   
int pidATGetLoopbackSec()
{
  return nLookBack * sampleTime / 1000;
}
