#include "uart.h"
#include "stm32f1xx.h"
#include "stm32f103xb.h"
#include "ctrl.h"
#include "main.h"
#include "PID.h"
#include <stdio.h>
#include <string.h>
#include "PS2.h"

#define M_Per_ROUND 188.4e-3

extern TIM_HandleTypeDef htim2;

/*state space*/
/*wheel distance */
float ml_cnt, mr_cnt; //mr on tim4; ml on tmi3
float rt_ml_cnt, rt_mr_cnt; // real time distance
/*wheel speed */
float ml_vel, mr_vel; //dps
/*car speed modified*/
float vel,dir;
/*PID bias*/
float ml_v_pid_bias,ml_x_pid_bias;
PID ml_v_PID, ml_x_PID;
/*TIM2 mode*/
uint8_t TIM2_MODE;

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
	if(TIM2_MODE == 1){
		TIM2->CCR1 = (uint16_t)yaw;
		TIM2->CCR2 = (uint16_t)pit;
	}
}

void ctrl_stepper(float dx){
	static float x;//current stepper pos in mm
	#define step_deg 18 //deg per pulse
	#define step_dx  0.5f //mm per pulse
	#define step_vel 20 //steps per sec (maximum)
	if(TIM2_MODE == 0){
		Stepper1 = sw_F;
		float speed = (uint16_t)(dx /step_dx);//set to ARR to change speed
		TIM2->CCR1 = 9999;
	}
}

void ctrl_init(void){
	//main freq 72M
	
	printf("PIDsize %d",sizeof(PID));
	PID_Init(&ml_v_PID,0,0,0,5,1);
	//motor freq 25K
	
	//deck T=20ms(2000), t=0.5~2.5ms(50-250)
}

void ctrl_switch_TIM2_Mode(uint8_t mode){
	/*
	mode = 0: using deck; 
	mode = 1: using stepper;
	*/
	if(mode>1)mode = 1;
	if(mode!=TIM2_MODE){
		if(mode){
			TIM2->ARR = 1999;//deck works on 50Hz
			TIM2->CCMR1 = 0x00006868; //reset to CRR	
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
			HAL_TIM_Base_Stop_IT(&htim2);
		}else{
			TIM2->ARR = 19999;//stepper works on 5Hz
			TIM2->CCMR1 = 0x00006868; //reset to 0
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
			HAL_TIM_Base_Start_IT(&htim2);
		}
	}
	TIM2_MODE= mode;
	return;
}

float pit,yaw;

void ctrl_exe(void){
	/*100Hz 180000 for TIM1_CNT*/
	static uint8_t CTRL_FLASH;
	CTRL_FLASH++;
	if(CTRL_FLASH==50){
		CTRL_FLASH=0;
		LED_B = !LED_B;
	}
	
	/*get remote */
	dir = ((float)get_sbus(1)/2000.0f * 2.0f -1.0f) * (vr_C/1000.0f);
	vel = ((float)get_sbus(2)/2000.0f * 2.0f -1.0f) * (vr_D/1000.0f);
	pit = (float)get_sbus(3)/3000.0f;
	yaw = (float)get_sbus(4)/2000.0f * 2.0f - 1.0f;
	float dx = (float)get_sbus(3)/2000.0f * 2.0f - 1.0f;
	
	static uint8_t PS2_DBG;
	PS2_DBG++;
	if(PS2_DBG==3){
		PS2_DBG=0;
		LED_R = !LED_R;
		PS2_ReadData();
	}
	
	/*get sensors */
	rt_ml_cnt = get_ml_cnt();
	ml_vel = get_ml_vel();
	
	/*get PID correction*/
	ml_x_pid_bias = PID_exe(&ml_x_PID, dir+vel, ml_cnt);
	ml_v_pid_bias = PID_exe(&ml_v_PID, dir+vel, ml_vel);
	
	/*switch*/
	Drv_EN = sw_D;
	SW1 = sw_H;
	SW2 = sw_F;
	
	/*motor*/
	ctrl_switch_TIM2_Mode(sw_E);
	ctrl_motor(vel-dir, vel+dir);
	ctrl_deck(yaw*180,pit*90);
	ctrl_stepper(dx*100);
	
	
	
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
	if(TIM4->CR1 & 0x10)
		ml_cnt--;
	else
		ml_cnt++;
}

