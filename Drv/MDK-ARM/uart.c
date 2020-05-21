#include "uart.h"
#include "stdio.h"
#include "main.h"
#include "string.h"
#include "TJMst.h"

SBUS_Pack RX;
uint8_t RX_CNT;
uint8_t RX_FLAG;
extern UART_HandleTypeDef huart3;

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

float set_tmp(float tmp){
	float temp = tmp;
	
	/*debug ADC1 val*/
	uint32_t adc_val_sum = 0;
	for(int i=0;i<50;i++){
		adc_val_sum+=ADC_VAL[2*i];
	}
	temp = (float)adc_val_sum / 50 * 0.00958167;
	
	
	
	return temp;
}

uint32_t console_exe(void){
	LED_B = !LED_B;
	
	if(!memcmp(RX_BUFF,"tmp",2))printf("tmp %.3f\r\n", set_tmp(0));
	if(!memcmp(RX_BUFF,"BAT",3))printf("BAT %.1f\r\n", BAT_Check());
	if(!memcmp(RX_BUFF,"TEMP",4))printf("TEMP %.1f\r\n", TEMP_Check());
	memset(RX_BUFF,0,DMA_SIZE);
	
	HAL_UART_Receive_DMA(&huart3,RX_BUFF,DMA_SIZE);
	
	return 1;
}

