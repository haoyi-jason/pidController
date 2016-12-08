#ifndef _PID_H_
#define _PID_H_

#include <aducm360.h>
#include "config.h"

#define AUTOMATIC       1
#define MANUAL          0

#define DIRECT          0
#define REVERSE         1

enum control_type_e{
  AT_PI,
  AT_PID
};




void pidInit(double *input,
             double *output,
             double *setPoint,
             double Kp,
             double Ki,
             double Kd,
             int controlDir);

void pidSetMode(int mode);
uint8_t pidCompute();
void pidSetOutputLimits(double,double);
void pidSetTunings(double,double,double);
void pidSetControllerDirection(int);
void pidSetSampleTime(int);
double pidGetKp();
double pidGetKi();
double pidGetKd();
int pidGetMode();
int pidGetDirection();
void initialize();

void pidSetTmrUpdate();
void pidProcess(double *in, double *out); 
void pidSetPoint(double v);
void pidSetTimer();
void pidProcessInit();
void pidSetPID(double p,double i,double d);
void pidSetPoint(double v);

void pidATuneInit(double *in, double *out);
void pidATFinishUp();
void pidATSetLoopbackSec(int val);
void pidATSetNoiseBand(double band);
void pidATSetOutputStep(double step);
void pidATCancel();
int pidATRuntime();
double pidATGetKp();
double pidATGetKi();
double pidATGetKd();
double pidGetOutput();

void pidInit2(double Kp, double Ki,double Kd, int controlDir);
void pidSetInput(double v);
float pidProcess2(double in);
bool pidATuneGetState();
void pidATuneSetState(bool s);
double pitATGetOutput();
#endif