#include "PS2.h"
#include "main.h"

uint16_t Handkey;
uint8_t Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint16_t MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};

static void PS2_delay_us(uint16_t us){
	for(;us>0;us--){
		for(int delay = 72 ; delay>0; delay--){};
	}
}	

void PS2_Init(void)
{
	
}

uint8_t PS2_WRByte(uint8_t byte){
	static uint32_t MPU_WR_TIMEOUT=168000;
	
	MPU_WR_TIMEOUT=168000;
	while(!(SPI_PS2->SR&0x0002)){
		if(MPU_WR_TIMEOUT--==0)return 0;
	}
	SPI_PS2->DR = byte;
	while(!(SPI_PS2->SR&0x0001)){
		if(MPU_WR_TIMEOUT--==0)return 0;		
	}
	byte = SPI_PS2->DR;
	PS2_delay_us(16);
	
	return byte;
}

uint8_t PS2_isRed(void)
{
	CS_PS2 = 0;
	PS2_WRByte(0x01);
	PS2_WRByte(0x42);
	CS_PS2 = 1;
	if( Data[1] == 0X73)   
		return 0 ;
	else 
		return 1;

}

//��ȡ�ֱ�����
void PS2_ReadData(void)
{
	volatile uint8_t byte=0;
	volatile uint16_t ref=0x01;
	CS_PS2 = 0;
	PS2_WRByte(0x01);  //��ʼ����
	PS2_WRByte(0x42);  //��������
	for(byte=2;byte<9;byte++)          //��ʼ��������
	{
		Data[byte] = PS2_WRByte(0x00);
		
    PS2_delay_us(16);
	}
	CS_PS2 = 1;
}

//�Զ�������PS2�����ݽ��д���,ֻ����������  
//ֻ��һ����������ʱ����Ϊ0�� δ����Ϊ1
uint8_t PS2_DataKey()
{
	uint8_t index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0�� δ����Ϊ1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //û���κΰ�������
}

//�õ�һ��ҡ�˵�ģ����	 ��Χ0~256
uint8_t PS2_AnologData(uint8_t button)
{
	return Data[button];
}

//������ݻ�����
void PS2_ClearData()
{
	uint8_t a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(uint8_t motor1, uint8_t motor2)
Description: �ֱ��𶯺�����
Calls:		 void PS2_WRByte(uint8_t CMD);
Input: motor1:�Ҳ�С�𶯵�� 0x00�أ�������
	   motor2:�����𶯵�� 0x40~0xFF �������ֵԽ�� ��Խ��
******************************************************/
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
	CS_PS2 = 0;
	PS2_delay_us(16);
    PS2_WRByte(0x01);  //��ʼ����
	PS2_WRByte(0x42);  //��������
	PS2_WRByte(0X00);
	PS2_WRByte(motor1);
	PS2_WRByte(motor2);
	PS2_WRByte(0X00);
	PS2_WRByte(0X00);
	PS2_WRByte(0X00);
	PS2_WRByte(0X00);
	CS_PS2 = 1;
	PS2_delay_us(16);  
}
//short poll
void PS2_ShortPoll(void)
{
	CS_PS2 = 0;
	PS2_delay_us(16);
	PS2_WRByte(0x01);  
	PS2_WRByte(0x42);  
	PS2_WRByte(0X00);
	PS2_WRByte(0x00);
	PS2_WRByte(0x00);
	CS_PS2 = 1;
	PS2_delay_us(16);	
}
//��������
void PS2_EnterConfing(void)
{
    CS_PS2 = 0;
	PS2_delay_us(16);
	PS2_WRByte(0x01);  
	PS2_WRByte(0x43);  
	PS2_WRByte(0X00);
	PS2_WRByte(0x01);
	PS2_WRByte(0x00);
	PS2_WRByte(0X00);
	PS2_WRByte(0X00);
	PS2_WRByte(0X00);
	PS2_WRByte(0X00);
	CS_PS2 = 1;
	PS2_delay_us(16);
}
//����ģʽ����
void PS2_TurnOnAnalogMode(void)
{
	CS_PS2 = 0;
	PS2_WRByte(0x01);  
	PS2_WRByte(0x44);  
	PS2_WRByte(0X00);
	PS2_WRByte(0x01); //analog=0x01;digital=0x00  ������÷���ģʽ
	PS2_WRByte(0x03); //Ox03�������ã�������ͨ��������MODE������ģʽ��
				   //0xEE������������ã���ͨ��������MODE������ģʽ��
	PS2_WRByte(0X00);
	PS2_WRByte(0X00);
	PS2_WRByte(0X00);
	PS2_WRByte(0X00);
	CS_PS2 = 1;
	PS2_delay_us(16);
}
//������
void PS2_VibrationMode(void)
{
	CS_PS2 = 0;
	PS2_delay_us(16);
	PS2_WRByte(0x01);  
	PS2_WRByte(0x4D);  
	PS2_WRByte(0X00);
	PS2_WRByte(0x00);
	PS2_WRByte(0X01);
	CS_PS2 = 1;
	PS2_delay_us(16);	
}
//��ɲ���������
void PS2_ExitConfing(void)
{
    CS_PS2 = 0;
	PS2_delay_us(16);
	PS2_WRByte(0x01);  
	PS2_WRByte(0x43);  
	PS2_WRByte(0X00);
	PS2_WRByte(0x00);
	PS2_WRByte(0x5A);
	PS2_WRByte(0x5A);
	PS2_WRByte(0x5A);
	PS2_WRByte(0x5A);
	PS2_WRByte(0x5A);
	CS_PS2 = 1;
	PS2_delay_us(16);
}
//�ֱ����ó�ʼ��
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//��������ģʽ
	PS2_TurnOnAnalogMode();	//�����̵ơ�����ģʽ����ѡ���Ƿ񱣴�
	//PS2_VibrationMode();	//������ģʽ
	PS2_ExitConfing();		//��ɲ���������
}

