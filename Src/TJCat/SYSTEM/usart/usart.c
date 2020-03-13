#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
	USART3->DR = (u8) ch;      
	return ch;
}
#endif
 	
u8 USART_RX_BUF[USART_REC_LEN];

/*
PC11_U3RX
PC10_U3TX
*/
void USART3_Init(u32 bound){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
		
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}



void USART3_IRQHandler(void)
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
		Res =USART_ReceiveData(USART3);
		printf("%c",Res);   
  }
}
#ifdef SBUS_ON_UART5
/*
S-BUS
100kbps
9-Even-2
using RX on PD2(UART5)
*/

void UART5_Init(){
	GPIO_InitTypeDef GPIO;
	USART_InitTypeDef USART;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);
  GPIO.GPIO_Pin = GPIO_Pin_2;
	GPIO.GPIO_Mode = GPIO_Mode_AF;
	GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO.GPIO_OType = GPIO_OType_OD;
	GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOD,&GPIO);

	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART.USART_BaudRate = 100000;
	USART.USART_WordLength = USART_WordLength_8b;
	USART.USART_Parity = USART_Parity_Even;
	USART.USART_StopBits = USART_StopBits_2;
	USART.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART.USART_Mode = USART_Mode_Rx;
  USART_Init(UART5, &USART);
  USART_Cmd(UART5, ENABLE);
}

typedef struct{
	u8 raw[25];
	u8 ptr;
	u16 ch[18];
}SBUS_Pack;

SBUS_Pack RX,buff;

u8 cnt;
void UART5_IRQHandler(void){
	u8 res;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) == RESET)return;
  USART_ClearITPendingBit ( UART5, USART_IT_RXNE ); 
	res = UART5->DR;
	RX.raw[RX.ptr++]  = res;	//start a frame
	if(RX.ptr==25){
		if(RX.raw[0] == 0x0F && RX.raw[22] == 0x80){//frame is aligned
			RX.ptr = 0;
			if(RX.raw[23] & 0x30)return;//receiver read missed
			RX.ch[0] =  (s16)(RX.raw[2] & 0x07) << 8 	| RX.raw[1];
			RX.ch[0] -= 300;//ch raw 300~1700
			RX.ch[1] =  (s16)(RX.raw[3] & 0x3f) << 5 	| (RX.raw[2] >> 3);
			RX.ch[1] -= 300;//ch raw 300~1700
			RX.ch[2] =  (s16)(RX.raw[5] & 0x01) << 10 | ((s16)RX.raw[4] << 2) | (RX.raw[3] >> 6);
			RX.ch[2] -= 300;//ch raw 300~1700
			RX.ch[3] =  (s16)(RX.raw[6] & 0x0F) << 7 	| (RX.raw[5] >> 1);
			RX.ch[3] -= 300;//ch raw 300~1700
			//switch C on ch[4]
			RX.ch[4] =  (s16)(RX.raw[7] & 0x7F) << 4 	| (RX.raw[6] >> 4);
			RX.ch[5] =  (s16)(RX.raw[9] & 0x03) << 9 	| ((s16)RX.raw[8] << 1) | (RX.raw[7] >> 7);
			RX.ch[6] =  (s16)(RX.raw[10] & 0x1F) << 6 | (RX.raw[9] >> 2);
			RX.ch[7] =  (s16)(RX.raw[11] 				<< 3) | (RX.raw[10] >> 5);
			
			//switch B on ch[8]
			RX.ch[8] =  (s16)(RX.raw[13] & 0x07)	<< 8 	| RX.raw[12];
			//switch A on ch[9]
			RX.ch[9] =  (s16)(RX.raw[14] & 0x3f) 	<< 5 	| (RX.raw[13] >> 3);
			RX.ch[10] = (s16)(RX.raw[16] & 0x01) 	<< 10 | ((s16)RX.raw[15] << 2) | (RX.raw[14] >> 6);
			RX.ch[11] = (s16)(RX.raw[17] & 0x0F) 	<< 7 	| (RX.raw[16] >> 1);
			RX.ch[12] = (s16)(RX.raw[18] & 0x7F) 	<< 4 	| (RX.raw[17] >> 4);
			RX.ch[13] = (s16)(RX.raw[20] & 0x03) 	<< 9 	| ((s16)RX.raw[19] << 1) | (RX.raw[18] >> 7);
			RX.ch[14] = (s16)(RX.raw[21] & 0x1F) 	<< 6 	| (RX.raw[20] >> 2);
			RX.ch[15] = (s16)RX.raw[22] 					<< 3 	| (RX.raw[21] >> 5);
			/*
			AT9 Radiolink[edc]
			306 when started
			joy limit to 309~1690
			0 as high, 2000 as low
			*/
		}else{//frame is not aligned, shift 1 byte
			for(res=0; res<24; res++)RX.raw[res] = RX.raw[res+1];
			RX.ptr = 24;
		}
	}
}

u16 get_sbus(u8 ch){
	switch(ch){
		case 1: return RX.ch[0];
		case 2: return RX.ch[1];
		case 3: return RX.ch[2];
		case 4: return RX.ch[3];
		case 10: return RX.ch[9];
	}
}

#endif


