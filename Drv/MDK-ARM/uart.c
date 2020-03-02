#include "uart.h"
#include "stdio.h"
#include "main.h"
#include "string.h"

SBUS_Pack RX,buff;
char RX_BUFF[30];
uint8_t RX_CNT;
uint8_t RX_FLAG;

/* basic support function*/
#define CONSOLE_UART USART3
#define RXER_UART USART2

#pragma import(__use_no_semihosting)             
FILE __stdout;       
struct __FILE 							{	int handle; 			}; 
void _sys_exit(int x)			 	{ x = x; 						} 
int fputc(int ch, FILE *f){
	while((CONSOLE_UART->SR&0X40)==0);
	CONSOLE_UART->DR = (uint8_t) ch;      
	return ch;
}
/* end support*/

uint16_t get_sbus(uint8_t ch){
	if(ch>16)return 1000;
	if(ch<1 )return 1000;
	return RX.ch[ch-1];
}

static float RX_DATA;

void set_tmp(float tmp){
	RX_DATA = tmp;
}

uint32_t io_exe(void){
	if(!RX_FLAG)return 0;
	LED_B = !LED_B;
	
	if(!memcmp(RX_BUFF,"tm",2))printf("tmp %f\r\n", RX_DATA);
	memset(RX_BUFF,0,30);
	RX_CNT = 0;
	RX_FLAG = 0;
	
	return 1;
}

