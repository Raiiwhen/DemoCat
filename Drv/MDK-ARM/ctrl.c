#include "uart.h"
#include "stm32f1xx.h"
#include "stm32f103xb.h"
#include "ctrl.h"
#include "main.h"

#define sw_D (get_sbus(5)>1000 ? 1 :0 )
#define sw_E (get_sbus(6)>1000 ? 1 :0 )
#define sw_F (get_sbus(7)>1000 ? 1 :0 )
#define sw_H (get_sbus(6)>1000 ? 1 :0 )

float ml_cnt, mr_cnt; //mr on tim4; ml on tmi3
float ml_vel, mr_vel; //dps
float vel,dir;

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
		TIM1->CCR3 = (uint16_t)(ml*2000);
		TIM1->CCR4 = 0;
	}else{
		if(ml<-1)ml=-1;
		TIM1->CCR3 = 0;
		TIM1->CCR4 = (uint16_t)(-ml*2000);
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

void ctrl_init(void){
	//main freq 72M
	
	//motor freq 25K
	
	//deck T=20ms(2000), t=0.5~2.5ms(50-250)
}


void ctrl_exe(void){
	/*get remote */
	dir = ((float)get_sbus(1)/2000.0f * 2.0f -1.0f) * (get_sbus(7)/1000.0f);
	vel = ((float)get_sbus(2)/2000.0f * 2.0f -1.0f) * (get_sbus(8)/1000.0f);
	float pit = (float)get_sbus(3)/2000.0f;
	float yaw = (float)get_sbus(4)/2000.0f * 2.0f - 1.0f;
	
	/*get feedback */
	mr_cnt += TIM3->CNT / ((TIM3->CR1&0x10)?-5.0f:5.0f); // rounds changed (round)
	mr_vel = TIM3->CNT;
	mr_cnt = 0;
	ml_cnt += (TIM4->CNT - 121)/2.5f; // rounds changed (round)
	ml_vel = ml_cnt * 360.0f * 1000.0f; //dps
	ml_cnt = 0;
	
	/*report msg*/
	set_tmp(mr_vel);
	
	/*execute*/
	Drv_EN = sw_D;
	ctrl_motor(vel-dir, vel+dir);
	ctrl_deck(yaw*180,pit*90);
	
	Spk_EN = sw_H;
}


void encoder3_update(void){
	LED_B = !LED_B;
	if(TIM3->CR1 & 0x10)
		mr_cnt+=5;
	else
		mr_cnt-=5;
}

void encoder4_update(void){
	float tmp = TIM4->CCR1;
	if(tmp>121)
		ml_cnt+=5;
	else
		ml_cnt-=5;
}

