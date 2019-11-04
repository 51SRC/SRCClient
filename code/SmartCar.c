/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC15F4K60S4 系列 定时器1用作串口1的波特率发生器举例------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966-------------------------*/
/* --- Web: www.STCMCU.com --------------------------------------------*/
/* --- Web: www.GXWMCU.com --------------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序        */
/* 如果要在文章中应用此代码,请在文章中注明使用了STC的资料及程序        */
/*---------------------------------------------------------------------*/

#include "DeviceAction.h"
#include "Delay.h"
#include "Timer.h"
#include "Public.h"



#define FOSC 11059200L       					   //系统频率
#define BAUD 115200            					 //串口波特率
	
#define S2RI  0x01              //S2CON.0
#define S2TI  0x02              //S2CON.1
#define S2RB8 0x04              //S2CON.2
#define S2TB8 0x08              //S2CON.3

#define S2_S0 0x01              //P_SW2.0


/*注：在进行正反转切换的时候最好先刹车0.1S以上再反转，否则有可能损坏驱动器。在PWM为100%时，如果要切换电机方向，必须先刹车0.1S以上再给反转信号。*/

bit	B_TX1_Busy;	//发送忙标志

bit MOTORRUNING = 1;
bit Engine_Status = 0;//引擎开关状态
bit Auto_Driver = 0;//自动驾驶状体

 uint8 SRCHeader = 0x7E;
 uint8 SRCTail = 0x7E;
 uint8 SRCDeviceID = 0x33;
 uint8 SRCCommunicationType = 0x01;


uint8 DATA_LENGTH = 9;
uint8 CURRENT_LENGTH=0;
uint8 CountTotle = 0;

uint8 DATA_GET[]=  { 0x7E, 0, 0, 0, 0, 0, 0, 0, 0x7E};


void SendData(char *s);
void UART_R(void);                               //接受数据
void ResponseData(unsigned char *RES_DATA);
void SendAckData(unsigned char *RES_DATA);
void UART2_Init(void);
void Device_Init(void);
void VehicleDiagnosis(unsigned int distance, unsigned char level);

 //摄像头   白负  黑正

void main()
{
    Device_Init();
    UART2_Init();
		Timer0_Init();
		Timer1_Init();
		//Timer4_Init();//定时器用于计算超声波
	  WDT_CONTR = 0x07;       //看门狗定时器溢出时间计算公式: (12 * 32768 * PS) / FOSC (秒)
                            //设置看门狗定时器分频数为32,溢出时间如下:
                            //11.0592M : 9s
                            //18.432M  : 8s
                            //20M      : 5s
    WDT_CONTR |= 0x20;      //启动看门狗

    while(1) {
			
			WDT_CONTR |= 0x10;  //喂狗程序


			
			
		};
}




void Device_Init(void) {
	
		P0M1 = 0;	P0M0 = 0;	//设置为准双向口
		P1M1 = 0;	P1M0 = 0;	//设置为准双向口
		P2M1 = 0;	P2M0 = 0;	//设置为准双向口
		P3M1 = 0;	P3M0 = 0;	//设置为准双向口
		P4M1 = 0;	P4M0 = 0;	//设置为准双向口
		P5M1 = 0;	P5M0 = 0;	//设置为准双向口
		P6M1 = 0;	P6M0 = 0;	//设置为准双向口
		P7M1 = 0;	P7M0 = 0;	//设置为准双向口
		
		P1M1 &= ~(0xD8);	  //P1.4 P1.3 设置为推挽输出    P1.6 P1.7 
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

    S2CON = 0x50;               //8位可变波特率
    T2L = (65536 - (FOSC/4/BAUD));   //设置波特率重装值
    T2H = (65536 - (FOSC/4/BAUD))>>8;
    AUXR = 0x14;                //T2为1T模式, 并启动定时器2
    IE2 = 0x01;                 //使能串口2中断
    EA = 1;
		B_TX1_Busy = 0;
		IP2 |=0x01;
}


/*----------------------------
UART 中断服务程序
-----------------------------*/
void UART2_interrupt() interrupt 8 using 1
{

	 if (S2CON & S2RI)
    {
        S2CON &= ~S2RI;         //清除S2RI位
         UART_R();
    }
    if (S2CON & S2TI)
    {
        S2CON &= ~S2TI;         //清除S2TI位
        B_TX1_Busy = 0;               //清忙标志
    }
}


/*----------------------------
发送串口数据
----------------------------*/

void  SendData(char *s)
{

    unsigned int i=0;

    for(i=0; i<DATA_LENGTH; i++)
    {
        S2BUF =s[i];
					while(!(S2CON & S2TI));
						S2CON &= ~S2TI;         //清除S2TI位
    }
}


//串口  接收到的数据

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
		//tudo 校验错误
		if(  RES_DATA[4]== 0x01 && (CheckData(RES_DATA) == RES_DATA[DATA_LENGTH-2])) {
				switch(RES_DATA[3]){
					case 0x00:{//心跳包
						if( RES_DATA[5]==0x00 && RES_DATA[6]==0x00){
							SendAckData(RES_DATA);
							Led_Actions_NumAndMS(1,80);
						}
						break;
					};
					case 0x01:{//转弯和角度
						
						break;
					};
					case 0x02:{//喇叭
						if( RES_DATA[6]==0x02){
							 Buzzer_Actions_Status(0);
							SendAckData(RES_DATA);
						}else if( RES_DATA[6]==0x01){
							Buzzer_Actions_Status(1);
							SendAckData(RES_DATA);
						}
						break;
					};
					case 0x03:{//灯
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
