#include "delay.h"
#include "DHT11.h"
#include "PUBLIC.H"


sbit DQ=P0^0;

void DHT11_Rst(void);
unsigned char DHT11_Check(void);
unsigned char DHT11_Read_Bit(void);
unsigned char DHT11_Read_Byte(void);
unsigned char DHT11_Read_Data(unsigned char *temp,unsigned char *humi);

/*
unsigned char DHT11()
{
	unsigned char humi=0x00,temp=0x00;

	if(DHT11_Read_Data(&temp,&humi)==0)
	{
		return 0;
	}
	else
		return 1;
}
*/


//复位DHT11
void DHT11_Rst(void)	   
{                 
	DQ=0;           	//拉低DQ
    DELAY_MS(20);    	//拉低至少18ms
    DQ=1; 	            //DQ=1 
	Delay30us();     	//主机拉高20~40us
}

//等待DHT11的回应
//返回1:未检测到DHT11的存在
//返回0:存在
unsigned char DHT11_Check(void) 	   
{   
	unsigned char retry=0; 
    while(DQ&&retry<100)//DHT11会拉低40~80us
	{
		retry++;
		Delay1us();
	};	 
	if(retry>=100)return 1;
	else retry=0;
    while (!DQ&&retry<100)//DHT11拉低后会再次拉高40~50us
	{
		retry++;
		Delay1us();
	};
	if(retry>=100)return 1;	    
	return 0;
}

//从DHT11读取一个位
//返回值：1/0
unsigned char DHT11_Read_Bit(void) 			 
{
 	unsigned char retry=0;
	while(DQ&&retry<100)//等待变为低电平
	{
		retry++;
		Delay1us();
	}
	retry=0;
	while(!DQ&&retry<100)//等待变高电平
	{
		retry++;
		Delay1us();
	}
	Delay50us();//等待50us
	if(DQ)return 1;
	else return 0;		   
}

//从DHT11读取一个字节
//返回值：读到的数据
unsigned char DHT11_Read_Byte(void)    
{        
    unsigned char i,dat;
    dat=0;
	for(i=0;i<8;i++) 
	{
   		dat<<=1; 
	    dat|=DHT11_Read_Bit();
    }						    
    return dat;
}

//从DHT11读取一次数据
//temp:温度值(范围:0~50°)
//humi:湿度值(范围:20%~90%)
//返回值：0,正常;1,读取失败
unsigned char DHT11_Read_Data(unsigned char *temp,unsigned char *humi)    
{        
 	unsigned char buf[5];
	unsigned char i;
	DHT11_Rst();
	if(DHT11_Check()==0)
	{
		for(i=0;i<5;i++)//读取40位数据
		{
			buf[i]=DHT11_Read_Byte();
		}
		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
		{
			*humi=buf[0];	
			*temp=buf[2];
		}
	}
	else return 1;
	return 0;	    
}


