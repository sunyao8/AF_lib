#ifndef __485ANN_H
#define __485ANN_H			 
#include "sys.h"	 
#include "rs485.h"
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "24cxx.h"
#include "includes.h" 	 
#include <math.h>
#include "adc.h"
#include "timer.h"								  

	  		  	
 
 typedef struct  
{ 
  u8 start;
  u8 myid;      //��������ID��
  u8 source;
  u8 destination; //Ŀ�ĵ�����
  u8 send;      //�Ƿ��Ƿ�������1Ϊ�ǣ�0Ϊ����
  u8 relay;    //�ڼ��������
  u8 message;     //������Ϣ
  u8 master;      //��������
  u8 end;   
}box;
#define LEN 7

#define MASTER_TASK_PRIO       			3 

//#define RS485_TX_EN		PGout(9)	//485ģʽ����.0,����;1,����.��������
#define RS485_TX_EN		PBout(15)	//485ģʽ����.0,����;1,����.��������
//����봮���жϽ��գ��벻Ҫע�����º궨��
#define EN_USART2_RX 	1			//0,������;1,����.


 //////////////////////////////////////////////////////////////////// 



void modfiy_token_array(u8,u8);
void turn_master_id(u8);
void initmybox(u8);
void TIM3_Int_Init(u16,u16);
void order_trans_rs485(u8,u8,u8,u8,u8);
int rs485_trans_order(u8 *);
#endif	   
















