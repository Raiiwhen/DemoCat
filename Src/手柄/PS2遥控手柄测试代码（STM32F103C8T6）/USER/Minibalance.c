#include "sys.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/ 
//�ֱ��ӿڳ�ʼ��    ����  DI->PA0 
//                 ���  DO->PA1    CS->PA2  CLK->PA3
//����1�Բ�����9600������յ�������
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
int main(void)
{ 
	Stm32_Clock_Init(9);            //=====ϵͳʱ������
	delay_init(72);                 //=====��ʱ��ʼ��
	LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
	KEY_Init();                     //=====������ʼ��
	uart_init(72,9600);           	//=====����1��ʼ��
	delay_ms(1000);                 //=====��ʱ�ȴ���ʼ���ȶ�
	PS2_Init();											//=====ps2�����˿ڳ�ʼ��
	PS2_SetInit();		 							//=====ps2���ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
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
