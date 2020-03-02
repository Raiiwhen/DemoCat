
#include "PinMap.h"

void PinMap_Init(void){
	GPIO_InitTypeDef GPIO;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO.GPIO_Mode = GPIO_Mode_OUT;
  GPIO.GPIO_OType = GPIO_OType_PP;
  GPIO.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO);
	GPIO_SetBits(GPIOC,GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	
	GPIO.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
  GPIO.GPIO_Mode = GPIO_Mode_IN;
  GPIO.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO);
}
