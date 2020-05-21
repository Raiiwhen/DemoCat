#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/ 
//手柄接口初始化    输入  DI->PA0 
//                 输出  DO->PA1    CS->PA2  CLK->PA3
//串口1以波特率9600输出接收到的数据
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
int main(void)
{ 
	Stm32_Clock_Init(9);            //=====系统时钟设置
	delay_init(72);                 //=====延时初始化
	LED_Init();                     //=====初始化与 LED 连接的硬件接口
	KEY_Init();                     //=====按键初始化
	uart_init(72,9600);           	//=====串口1初始化
	delay_ms(1000);                 //=====延时等待初始化稳定
	PS2_Init();											//=====ps2驱动端口初始化
	PS2_SetInit();		 							//=====ps2配置初始化,配置“红绿灯模式”，并选择是否可以修改
	delay_ms(1000);
	while(1)
		{		
			PS2_LX=PS2_AnologData(PSS_LX);    
			PS2_LY=PS2_AnologData(PSS_LY);
			PS2_RX=PS2_AnologData(PSS_RX);
			PS2_RY=PS2_AnologData(PSS_RY);
			PS2_KEY=PS2_DataKey();	
			printf("%d     PS2_LX:",PS2_LX);
			printf("%d     PS2_LY:",PS2_LY);
		  printf("%d     PS2_RX:",PS2_RX);
			printf("%d     PS2_RY:",PS2_RY);
			printf("%d \r\nPS2_KEY:",PS2_KEY);
			delay_ms(100);
		} 
}
