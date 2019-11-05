#include "Delay.h"
#include <INTRINS.H>

// ����: ms,Ҫ��ʱ��ms��, ����ֻ֧��1~255ms.  11059200UL�Զ���Ӧ��ʱ��.

void DELAY_MS(unsigned char ms){
    unsigned int i;
		do{
	      i = 11059200UL / 13000;
		  while(--i)	;
     }while(--ms);
}

void Delay10us()		//@11.0592MHz
{
	unsigned char i;

	_nop_();
	i = 25;
	while (--i);
}

void Delay1us()		//@11.0592MHz
{
	_nop_();
	_nop_();
	_nop_();
}
void Delay30us()		//@11.0592MHz
{
	unsigned char i;
	_nop_();
	_nop_();
	i = 80;
	while (--i);
}

void Delay50us()		//@11.0592MHz
{
	unsigned char i, j;

	_nop_();
	i = 1;
	j = 134;
	do
	{
		while (--j);
	} while (--i);
}

