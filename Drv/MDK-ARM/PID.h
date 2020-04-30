#ifndef _PID_H_
#define _PID_H_

#include <math.h>

typedef struct{
	float err;
	float kP,P;
	float kI,I;
	float kD,D;
	float dt;
	float limit_max;
}PID;

void PID_Init(PID* obj, float kP, float kI, float kD, float dt, float limit_max);
float PID_exe(PID* obj, float exp, float real);

#endif
