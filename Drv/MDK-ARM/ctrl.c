#include "uart.h"
#include "stm32f1xx.h"
#include "stm32f103xb.h"
#include "ctrl.h"
#include "main.h"
#include "PID.h"

#define M_Per_ROUND 188.4e-3

/*state space*/
/*wheel distance */
float ml_cnt, mr_cnt; //mr on tim4; ml on tmi3
float rt_ml_cnt, rt_mr_cnt; // real time distance
/*wheel speed */
float ml_vel, mr_vel; //dps
/*car speed modified*/
float vel,dir;
/*PID bias*/
float mr_add, ml_add;
PID mr_PID, ml_PID;

void ctrl_motor(float mr, float ml){
	if(mr>0){
		if(mr>1)mr=1;
		TIM1->CCR1 = (uint16_t)(mr*2000);
		TIM1->CCR2 = 0;
	}else{
		if(mr<-1)mr=-1;
		TIM1->CCR1 = 0;
		TIM1->CCR2 = (uint16_t)(-mr*2000);
	}
	ml *= -1;
	if(ml>0){
		if(ml>1)ml=1;
		TIM1->CCR3 = 0;
		TIM1->CCR4 = (uint16_t)(ml*2000);
	}else{
		if(ml<-1)ml=-1;
		TIM1->CCR3 = (uint16_t)(-ml*2000);
		TIM1->CCR4 = 0;
	}
}

void ctrl_deck(float _yaw, float _pit){
	float yaw = _yaw / 180;
	float pit = _pit / 90;
	
	if(yaw>1)yaw=1; if(yaw<-1)yaw=-1;
	if(pit>1)pit=1; if(pit<0)pit=0;
	
	yaw *= 100; yaw+= 150;	//+-180 => +-1 => 150+-100
	pit *= 200; pit+= 50;		//+90 	=> +1	 =>	50+200
	
	TIM2->CCR1 = (uint16_t)yaw;
	TIM2->CCR2 = (uint16_t)pit;
}

void ctrl_stepper(float dx){
	static float x;//current stepper pos
	#define step_deg 18 //deg per pulse
	#define step_dx  0.5f //cm per pulse
	#define step_vel 20 //deg per sec (maximum)
	
	
}

void ctrl_init(void){
	//main freq 72M
	
	//motor freq 25K
	
	//deck T=20ms(2000), t=0.5~2.5ms(50-250)
}

void ctrl_switch_TIM2_Mode(uint8_t mode){
	static uint8_t last_mode;
	/*
	mode = 0: using deck; 
	mode = 1: using stepper;
	*/
	if(mode>1)mode = 1;
	if(mode!=last_mode){
		if(mode){
			TIM2->ARR = 1999;//deck works on 50Hz
			TIM2->CCMR1 = 0x00006868; //reset to CRR
		}else{
			TIM2->ARR = 19999;//stepper works on 5Hz
			TIM2->CCMR1 = 0x00006060; //reset to 0
		}
	}
	last_mode= mode;
	return;
}

float debug;
void ctrl_exe(void){
	/*100Hz systme frequency*/
	
	/*get remote */
	dir = ((float)get_sbus(1)/2000.0f * 2.0f -1.0f) * (vr_C/1000.0f);
	vel = ((float)get_sbus(2)/2000.0f * 2.0f -1.0f) * (vr_D/1000.0f);
	float pit = (float)get_sbus(3)/3000.0f;
	float yaw = (float)get_sbus(4)/2000.0f * 2.0f - 1.0f;
	
	/*get sensors */
	rt_ml_cnt = get_ml_cnt();
	rt_mr_cnt = get_mr_cnt();
	ml_vel = get_ml_vel();
	
	/*get PID correction*/
	ml_add = PID_exe(&ml_PID, dir+vel, mr_cnt);
	
	/*report msg*/
	set_tmp(mr_vel);
	
	/*execute*/
	Drv_EN = sw_D;
	
	SW1 = sw_H;
	SW2 = sw_F;
	SW3 = sw_F;
	SW4 = sw_F;
	SW5 = sw_F;
	SW6 = sw_F;
	ctrl_motor(vel-dir, vel+dir);
	ctrl_deck(yaw*180,pit*90);
	
	
}


float get_mr_cnt(void){
	return 0;
}

float get_ml_cnt(void){
	float cnt = 0;
	cnt = (ml_cnt + ((short)TIM4->CNT-121)/121.0)*M_Per_ROUND;
	return cnt;
}

float get_ml_vel(void){
	static float last_pos;
	float rt_pos, rt_vel;
	
	rt_pos = get_ml_cnt();
	rt_vel = (rt_pos - last_pos)*100; //dps
	last_pos = rt_pos;
	
	return rt_vel;
}

void encoder3_update(void){
}

void encoder4_update(void){
	LED_B = !LED_B;
	if(TIM4->CR1 & 0x10)
		ml_cnt--;
	else
		ml_cnt++;
}

