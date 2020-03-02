#ifndef __UART_H
#define __UART_H

#include "stm32f1xx.h"

typedef struct{
	uint8_t raw[25];
	uint8_t ptr;
	uint16_t ch[18];
}SBUS_Pack;

uint16_t get_sbus(uint8_t ch);
void set_tmp(float tmp);
uint32_t io_exe(void);


#endif

