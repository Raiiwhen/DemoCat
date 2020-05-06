#ifndef _TJMST_H
#define _TJMST_H

#include "stm32f1xx_hal.h"

#define DMA_SIZE 128
extern uint8_t RX_BUFF[128];
extern uint8_t TX_BUFF[128];

float BAT_Check();
float TEMP_Check();
void mst_set(uint8_t mode);
void mst_execute();

#endif
