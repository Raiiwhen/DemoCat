#include "PID.h"

void PID_Init(PID* obj, float kP, float kI, float kD, float dt, float limit_max){
	obj->kP = kP;
	obj->kI = kI;
	obj->kD = kD;
	obj->dt = dt;
	obj->limit_max = limit_max;
}

float PID_exe(PID* obj, float exp, float real){
	float err = 0;
	float sum = 0;
	
	err = exp - real;
	obj->P = err * obj->kP;
	obj->I += err * obj->kI;
	if(obj->I > obj->limit_max)	obj->I = obj->limit_max;
	obj->D = (err - obj->err) * obj->kD * obj->dt;
	
	obj->err = err;
	sum = obj->P + obj->I + obj->D;
	
	return sum;
}
