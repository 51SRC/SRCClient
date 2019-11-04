/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC15F4K60S4 ϵ�� ��ʱ��1��������1�Ĳ����ʷ���������------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966-------------------------*/
/* --- Web: www.STCMCU.com --------------------------------------------*/
/* --- Web: www.GXWMCU.com --------------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ����STC�����ϼ�����        */
/* ���Ҫ��������Ӧ�ô˴���,����������ע��ʹ����STC�����ϼ�����        */
/*---------------------------------------------------------------------*/

#include "DeviceAction.h"
#include "Delay.h"
#include "Timer.h"
#include "Public.h"



#define FOSC 11059200L       					   //ϵͳƵ��
#define BAUD 115200            					 //���ڲ�����
	
#define S2RI  0x01              //S2CON.0
#define S2TI  0x02              //S2CON.1
#define S2RB8 0x04              //S2CON.2
#define S2TB8 0x08              //S2CON.3

#define S2_S0 0x01              //P_SW2.0


/*ע���ڽ�������ת�л���ʱ�������ɲ��0.1S�����ٷ�ת�������п���������������PWMΪ100%ʱ�����Ҫ�л�������򣬱�����ɲ��0.1S�����ٸ���ת�źš�*/

bit	B_TX1_Busy;	//����æ��־

bit MOTORRUNING = 1;
bit Engine_Status = 0;//���濪��״̬
bit Auto_Driver = 0;//�Զ���ʻ״��

 uint8 SRCHeader = 0x7E;
 uint8 SRCTail = 0x7E;
 uint8 SRCDeviceID = 0x33;
 uint8 SRCCommunicationType = 0x01;


uint8 DATA_LENGTH = 9;
uint8 CURRENT_LENGTH=0;
uint8 CountTotle = 0;

uint8 DATA_GET[]=  { 0x7E, 0, 0, 0, 0, 0, 0, 0, 0x7E};


void SendData(char *s);
void UART_R(void);                               //��������
void ResponseData(unsigned char *RES_DATA);
void SendAckData(unsigned char *RES_DATA);
void UART2_Init(void);
void Device_Init(void);
void VehicleDiagnosis(unsigned int distance, unsigned char level);

 //����ͷ   �׸�  ����

void main()
{
    Device_Init();
    UART2_Init();
		Timer0_Init();
		Timer1_Init();
		//Timer4_Init();//��ʱ�����ڼ��㳬����
	  WDT_CONTR = 0x07;       //���Ź���ʱ�����ʱ����㹫ʽ: (12 * 32768 * PS) / FOSC (��)
                            //���ÿ��Ź���ʱ����Ƶ��Ϊ32,���ʱ������:
                            //11.0592M : 9s
                            //18.432M  : 8s
                            //20M      : 5s
    WDT_CONTR |= 0x20;      //�������Ź�

    while(1) {
			
			WDT_CONTR |= 0x10;  //ι������


			
			
		};
}




void Device_Init(void) {
	
		P0M1 = 0;	P0M0 = 0;	//����Ϊ׼˫���
		P1M1 = 0;	P1M0 = 0;	//����Ϊ׼˫���
		P2M1 = 0;	P2M0 = 0;	//����Ϊ׼˫���
		P3M1 = 0;	P3M0 = 0;	//����Ϊ׼˫���
		P4M1 = 0;	P4M0 = 0;	//����Ϊ׼˫���
		P5M1 = 0;	P5M0 = 0;	//����Ϊ׼˫���
		P6M1 = 0;	P6M0 = 0;	//����Ϊ׼˫���
		P7M1 = 0;	P7M0 = 0;	//����Ϊ׼˫���
		
		P1M1 &= ~(0xD8);	  //P1.4 P1.3 ����Ϊ�������    P1.6 P1.7 
		P1M0 |=  (0xD8);
		P4M1 &= ~(0x20);
		P4M0 |=  (0x20);
		
		
		MOTORRUNING = 1;
		Engine_Status = 0;
		
    Buzzer_Actions_Status(1);
	  Led_Actions_Status(0);
	  InitMoter();
		Motor_Actions_Status(0,0);
		
}


void UART2_Init()
{

    P_SW2 &= ~S2_S0;            //S2_S0=0 (P1.0/RxD2, P1.1/TxD2)

    S2CON = 0x50;               //8λ�ɱ䲨����
    T2L = (65536 - (FOSC/4/BAUD));   //���ò�������װֵ
    T2H = (65536 - (FOSC/4/BAUD))>>8;
    AUXR = 0x14;                //T2Ϊ1Tģʽ, ��������ʱ��2
    IE2 = 0x01;                 //ʹ�ܴ���2�ж�
    EA = 1;
		B_TX1_Busy = 0;
		IP2 |=0x01;
}


/*----------------------------
UART �жϷ������
-----------------------------*/
void UART2_interrupt() interrupt 8 using 1
{

	 if (S2CON & S2RI)
    {
        S2CON &= ~S2RI;         //���S2RIλ
         UART_R();
    }
    if (S2CON & S2TI)
    {
        S2CON &= ~S2TI;         //���S2TIλ
        B_TX1_Busy = 0;               //��æ��־
    }
}


/*----------------------------
���ʹ�������
----------------------------*/

void  SendData(char *s)
{

    unsigned int i=0;

    for(i=0; i<DATA_LENGTH; i++)
    {
        S2BUF =s[i];
					while(!(S2CON & S2TI));
						S2CON &= ~S2TI;         //���S2TIλ
    }
}


//����  ���յ�������

void UART_R()
{
    DATA_GET[CURRENT_LENGTH]=S2BUF ;
    CURRENT_LENGTH++;
		CountTotle++;
		
    if(CURRENT_LENGTH==DATA_LENGTH && !B_TX1_Busy)
    {
				if(DATA_GET[0] == SRCHeader && DATA_GET[DATA_LENGTH-1] == SRCTail ){
					CountTotle = 0;
						CURRENT_LENGTH=0;
						B_TX1_Busy = 1;
						ResponseData(DATA_GET);
				}else {
				
				}
       
    }else if(	CURRENT_LENGTH==2 && DATA_GET[0]==SRCHeader && DATA_GET[1]==SRCHeader){
			CURRENT_LENGTH = 1;

		}
		
		if(CountTotle > DATA_LENGTH){
						IAP_CONTR = 0X60;
		}
		
		
		
}


void ResponseData(unsigned char *RES_DATA) {
	

	if(RES_DATA[1]== 0x33 &&  RES_DATA[2]== 0x01){
		//tudo У�����
		if(  RES_DATA[4]== 0x01 && (CheckData(RES_DATA) == RES_DATA[DATA_LENGTH-2])) {
				switch(RES_DATA[3]){
					case 0x00:{//������
						if( RES_DATA[5]==0x00 && RES_DATA[6]==0x00){
							SendAckData(RES_DATA);
							Led_Actions_NumAndMS(1,80);
						}
						break;
					};
					case 0x01:{//ת��ͽǶ�
						
						break;
					};
					case 0x02:{//����
						if( RES_DATA[6]==0x02){
							 Buzzer_Actions_Status(0);
							SendAckData(RES_DATA);
						}else if( RES_DATA[6]==0x01){
							Buzzer_Actions_Status(1);
							SendAckData(RES_DATA);
						}
						break;
					};
					case 0x03:{//��
						if( RES_DATA[6]==0x02){
						Led_Actions_Status(1);
							SendAckData(RES_DATA);
						}else if( RES_DATA[6]==0x01){
							Led_Actions_Status(0);
							SendAckData(RES_DATA);
						}
						break;
					};

					default:
						break;
					
				}
			}
		
		
		
		
		
	}
		
		B_TX1_Busy = 0;

}


void SendAckData(unsigned char *RES_DATA) {

    unsigned char DATA_SEND[]= { 0x7E, 0x00,0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x7E};

    DATA_SEND[0]= SRCHeader;
    DATA_SEND[1]= SRCDeviceID;
    DATA_SEND[2]= SRCCommunicationType;
    DATA_SEND[3]= RES_DATA[3];
    DATA_SEND[5]= RES_DATA[5];
    DATA_SEND[6]= RES_DATA[6];
    DATA_SEND[DATA_LENGTH-1]= SRCTail;
    DATA_SEND[7]= CheckData(DATA_SEND);

    SendData(DATA_SEND);

}
