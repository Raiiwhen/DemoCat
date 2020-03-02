#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f4xx.h"
#define TEST_CAN1_RX
typedef struct MOTOR_FEEDBACK{
	u8 motor_ID;
	float rpm[4];		//round per min
	float deg[4];	//0~360deg
	float mom[4];		//real force moment
}RAM_HORN;

void M2006_Init(void);
void M2006_Thr(float LF,float LB,float RF,float RB);
void M2006_Feedback(RAM_HORN* obj);

#endif


