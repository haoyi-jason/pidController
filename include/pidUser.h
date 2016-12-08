#ifndef _PIDUSER_H
#define _PIDUSER_H


enum {
  PID_IDLE,
  PID_AUTO,
  PID_TUNING,
};

int pidUserPoll(void);
int pidUserInit(void);


#endif