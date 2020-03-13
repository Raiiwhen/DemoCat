#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "PinMap.h"
#include "motor.h"

/*
测试信息:
接线：将电调接入CAN1总线，确保打开2个120Ω终端电阻开关；
注意！串口配置为115200 8N1
接收机接在UART5使用PD2作RX，A拨杆在低位时使用遥杆控制ID1~4电机，
接收机不接或者A拨杆在高位时电机刹车

现象
U3TX: 串口接收到一帧"Meow is alive\r\n"，若CAN可用则每次黄色LED闪烁都将受到一次ID1电机的反馈值。
U3TX: 串口发送任何数据总是立即收到相同的数据
XTAL: 红色LED每0.1秒闪烁一次，代表主while在运行；晶振正常；
CAN1TX: 黄色LED闪烁代表CAN1总线发送2帧油门帧；
CAN1RX: 蓝色LED闪烁一次代表进入了2000次CAN接收中断；
*/

float command[4];

int main(void)
{
	u8 t;
	u8 len;	
	static u16 times=0;  
	PinMap_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	USART3_Init(115200);
	UART5_Init();
	//LED_Init();		  		//初始化与LED连接的硬件接口  
	printf("Meow is alive\r\n");
	M2006_Init();
	M2006_Thr(0,0,0,0);
	delay_ms(500);
	while(1)
	{
		times%50 ? (1) : (LED_R=!LED_R);
		times++;
		if(get_sbus(10)>1900){
			M2006_Thr(((float)get_sbus(2)/700-1) + ((float)get_sbus(1)/700-1)*0.3,
								((float)get_sbus(2)/700-1) + ((float)get_sbus(1)/700-1)*0.3,
								-((float)get_sbus(2)/700-1) + ((float)get_sbus(1)/700-1)*0.3,
								-((float)get_sbus(2)/700-1) + ((float)get_sbus(1)/700-1)*0.3);
		}else{
			M2006_Thr(0,0,0,0);
		}
		delay_ms(20);
	}
}

