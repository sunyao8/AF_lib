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
  u8 myid;      //本电容箱ID号
  u8 source;
  u8 destination; //目的电容箱
  u8 send;      //是否是发送命令1为是，0为不是
  u8 relay;    //第几组电容器
  u8 message;     //开关信息
  u8 master;      //主机令牌
  u8 end;   
}box;
#define LEN 7

#define MASTER_TASK_PRIO       			3 

//#define RS485_TX_EN		PGout(9)	//485模式控制.0,接收;1,发送.开发板用
#define RS485_TX_EN		PBout(15)	//485模式控制.0,接收;1,发送.本工程用
//如果想串口中断接收，请不要注释以下宏定义
#define EN_USART2_RX 	1			//0,不接收;1,接收.


 //////////////////////////////////////////////////////////////////// 



void modfiy_token_array(u8,u8);
void turn_master_id(u8);
void initmybox(u8);
void TIM3_Int_Init(u16,u16);
void order_trans_rs485(u8,u8,u8,u8,u8);
int rs485_trans_order(u8 *);
#endif	   
















