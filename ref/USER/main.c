#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "PinMap.h"
#include "motor.h"

/*
������Ϣ:
���ߣ����������CAN1���ߣ�ȷ����2��120���ն˵��迪�أ�
ע�⣡��������Ϊ115200 8N1
���ջ�����UART5ʹ��PD2��RX��A�����ڵ�λʱʹ��ң�˿���ID1~4�����
���ջ����ӻ���A�����ڸ�λʱ���ɲ��

����
U3TX: ���ڽ��յ�һ֡"Meow is alive\r\n"����CAN������ÿ�λ�ɫLED��˸�����ܵ�һ��ID1����ķ���ֵ��
U3TX: ���ڷ����κ��������������յ���ͬ������
XTAL: ��ɫLEDÿ0.1����˸һ�Σ�������while�����У�����������
CAN1TX: ��ɫLED��˸����CAN1���߷���2֡����֡��
CAN1RX: ��ɫLED��˸һ�δ��������2000��CAN�����жϣ�
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
	//LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�  
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

