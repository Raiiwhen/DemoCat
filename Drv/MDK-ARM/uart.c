#include "uart.h"
#include "stdio.h"
#include "main.h"
#include "string.h"

SBUS_Pack RX,buff;
char RX_BUFF[30];
uint8_t RX_CNT;
uint8_t RX_FLAG;
uint32_t ADC_VAL[100];

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
	
	/*debug ADC1 val*/
	uint32_t adc_val_sum = 0;
	for(int i=0;i<50;i++){
		adc_val_sum+=ADC_VAL[2*i];
	}
	RX_DATA = (float)adc_val_sum / 50 / 4096 * 3.3 * 12.255;
}

uint32_t io_exe(void){
	if(!RX_FLAG)return 0;
	LED_B = !LED_B;
	
	if(!memcmp(RX_BUFF,"tm",2))printf("tmp %.1f\r\n", RX_DATA);
	memset(RX_BUFF,0,30);
	RX_CNT = 0;
	RX_FLAG = 0;
	
	return 1;
}

