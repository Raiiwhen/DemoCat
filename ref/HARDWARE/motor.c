#include "stm32f4xx_can.h"
#include "stdio.h"
#include "motor.h"
#include "PinMap.h"

RAM_HORN CLAW;

void M2006_Init(void){
	GPIO_InitTypeDef 				GPIO; 
	CAN_InitTypeDef        	CAN;
	CAN_FilterInitTypeDef  	CAN_Filter;
	NVIC_InitTypeDef  			NVIC_VECT;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	GPIO.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
	GPIO.GPIO_Mode = GPIO_Mode_AF;
	GPIO.GPIO_OType = GPIO_OType_PP;
	GPIO.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN);
	CAN.CAN_NART=ENABLE;
	CAN.CAN_Mode = CAN_Mode_Normal;
	//BaudRate = 42M/((TS1[3:0]	+TS2[2:0]	+3)*(BRP[9:0]	+1));
	//BaudRate = 42M/((5				+6				+3)*(2				+1));
	//M2006@C610 CAN Baudrate = 1M
	CAN.CAN_SJW  = CAN_SJW_1tq;
	CAN.CAN_BS1  = CAN_BS1_9tq;
	CAN.CAN_BS2  = CAN_BS2_4tq;
	CAN.CAN_Prescaler = 3;
	CAN_Init(CAN1, &CAN);
	
	CAN_Filter.CAN_FilterNumber=0;
	CAN_Filter.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_Filter.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_Filter.CAN_FilterIdHigh=0x0000;
	CAN_Filter.CAN_FilterIdLow=0x0000;
	CAN_Filter.CAN_FilterMaskIdHigh=0x0000;
	CAN_Filter.CAN_FilterMaskIdLow=0x0000;																																																																																																																																																																																																																																																																								
	CAN_Filter.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_Filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_Filter);
	
	NVIC_VECT.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_VECT.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_VECT.NVIC_IRQChannelSubPriority = 1;
	NVIC_VECT.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_VECT);
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);							
}

static CanRxMsg RxMessage;

void CAN1_RX0_IRQHandler(void){
	int i=0;
	static u16 RX_psr;
	short rpm, deg, mom;
	CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);	
	switch(RxMessage.StdId){
		case 0x201:	
			deg = (RxMessage.Data[0]<<8) + RxMessage.Data[1];
			rpm = (RxMessage.Data[2]<<8) + RxMessage.Data[3];
			mom = (RxMessage.Data[4]<<8) + RxMessage.Data[5];
			CLAW.rpm[0] = rpm;
			CLAW.deg[0] = 0.04395067757294591624954218044195 * deg;
			CLAW.mom[0] = mom;
			break;
//		case 0x202:			
//			deg = (RxMessage.Data[0]<<8) + RxMessage.Data[1];
//			rpm = (RxMessage.Data[2]<<8) + RxMessage.Data[3];
//			mom = (RxMessage.Data[4]<<8) + RxMessage.Data[5];
//			CLAW.rpm[1] = rpm;
//			CLAW.deg[1] = 0.04395067757294591624954218044195 * deg;
//			CLAW.mom[1] = mom;
//			break;
//		case 0x203:			
//			deg = (RxMessage.Data[0]<<8) + RxMessage.Data[1];
//			rpm = (RxMessage.Data[2]<<8) + RxMessage.Data[3];
//			mom = (RxMessage.Data[4]<<8) + RxMessage.Data[5];
//			CLAW.rpm[2] = rpm;
//			CLAW.deg[2] = 0.04395067757294591624954218044195 * deg;
//			CLAW.mom[2] = mom;
//			break;
//		case 0x204:			
//			deg = (RxMessage.Data[0]<<8) + RxMessage.Data[1];
//			rpm = (RxMessage.Data[2]<<8) + RxMessage.Data[3];
//			mom = (RxMessage.Data[4]<<8) + RxMessage.Data[5];
//			CLAW.rpm[3] = rpm;
//			CLAW.deg[3] = 0.04395067757294591624954218044195 * deg;
//			CLAW.mom[3] = mom;
//			break;
	}
	if(!(RX_psr%1000)){
		LED_B=!LED_B;
		#ifdef TEST_CAN1_RX
			printf("rpm%8.2f|",CLAW.rpm[0]);
			printf("deg%8.2f|",CLAW.deg[0]);
			printf("mom%8.2f|",CLAW.mom[0]);
			printf("\r\n");
		#endif
	}
	RX_psr++;
}

void M2006_Thr(float LF,float LB,float RF,float RB){
	CanTxMsg tx_message;
	short buff[4];
	static u16 TX_psr;
	if(TX_psr>3){
		LED_Y=1;
	}else{
		LED_Y=0;		
	}
	TX_psr==10 ? TX_psr=0 : TX_psr++;
	if(LF>1){
		buff[0] = 6000;
	}else if(LF<-1){
		buff[0] = -6000;
	}else{
		buff[0] = LF*6000;
	}
	if(LB>1){
		buff[1] = 6000;
	}else if(LB<-1){
		buff[1] = -6000;
	}else{
		buff[1] = LB*6000;
	}
	if(RF>1){
		buff[2] = 6000;
	}else if(RF<-1){
		buff[2] = -6000;
	}else{
		buff[2] = RF*6000;
	}
	if(RB>1){
		buff[3] = 6000;
	}else if(RB<-1){
		buff[3] = -6000;
	}else{
		buff[3] = RB*6000;
	}
	
	/*M2006 limit to ¡À6000 as ¡À6A; 70% - 450rpm - 90W - 6A - 1.7Nm*/
	
	tx_message.IDE = CAN_ID_STD;    
	tx_message.RTR = CAN_RTR_DATA; 
	tx_message.DLC = 0x08;    
	tx_message.StdId = 0x200;
	
	tx_message.Data[0] = (buff[0]&0xff00)>>8;     	
	tx_message.Data[1] = buff[0]&0x00ff;
	tx_message.Data[2] = (buff[1]&0xff00)>>8;     	
	tx_message.Data[3] = buff[1]&0x00ff;
	tx_message.Data[4] = (buff[2]&0xff00)>>8;     	
	tx_message.Data[5] = buff[2]&0x00ff;
	tx_message.Data[6] = (buff[3]&0xff00)>>8;     	
	tx_message.Data[7] = buff[3]&0x00ff;
	CAN_Transmit(CAN1,&tx_message);
}
