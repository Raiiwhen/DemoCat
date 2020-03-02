#ifndef _PINMAP_H_
#define _PINMAP_H_
#include "stm32f4xx.h"
#include "sys.h"

#define LED_R PCout(13)
#define LED_Y PCout(15)
#define LED_B PCout(14)
#define KEY_L PCin(3)
#define KEY_R PCin(2)
void PinMap_Init(void);

#endif
