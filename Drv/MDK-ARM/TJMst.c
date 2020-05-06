#include "TJMst.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

uint8_t RX_BUFF[DMA_SIZE];
uint8_t TX_BUFF[DMA_SIZE];

void float2bin(uint8_t pDst, float tmp){
	
	
}

float BAT_Check(){
	float tmp;
	uint32_t adc_val_sum = 0;
	for(int i=0;i<50;i++){
		adc_val_sum+=ADC_VAL[2*i];
	}
	tmp = (float)adc_val_sum / 50 * 0.00958167;
	
	return tmp;
}

float TEMP_Check(){
	float tmp;
	uint32_t adc_val_sum = 0;
	for(int i=0;i<50;i++){
		adc_val_sum+=ADC_VAL[2*i+1];
	}
	tmp = (1.43f - (float)adc_val_sum/50.0f/4096.0f*3.3f)/0.0043+25.0f;
	
	return tmp;
}

void mst_set(uint8_t mode){
	
	
}

void mst_pkgecho(){
	uint32_t temp;
	TX_BUFF[0] = 0x0c;
	TX_BUFF[1] = 0x0c;
	TX_BUFF[2] = 0x00;
	TX_BUFF[3] = 0x00;//CCR
	temp = (uint32_t)(TEMP_Check()*100);
	memcpy(TX_BUFF+4,(uint8_t*)&temp,4);
	temp = (uint32_t)(BAT_Check()*100);
	memcpy(TX_BUFF+8,(uint8_t*)&temp,4);
	
	for(int i =4;i<128;i++)TX_BUFF[3]+=TX_BUFF[i];
}

void mst_execute(){
	memset(TX_BUFF,DMA_SIZE,0);
	switch(RX_BUFF[0]){
		case 0x0c: mst_pkgecho();break;
		
		
		default: break;
	}
	LED_B = !LED_B;
	memset(RX_BUFF,DMA_SIZE,0);
	HAL_UART_Transmit_DMA(&huart3,TX_BUFF,DMA_SIZE);
}
