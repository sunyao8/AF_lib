
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
#include "485ANN.h"
//32

/////////////////////////UCOSII任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	
 			   
//LED任务
//设置任务优先级
#define TAKE_TASK_PRIO       			7 
//设置任务堆栈大小
#define TAKE_STK_SIZE  		    		64
//任务堆栈
OS_STK TAKE_TASK_STK[TAKE_STK_SIZE];
//任务函数
void Heartbeat_task(void *pdata);

//接收任务
//设置任务优先级
#define Receive_TASK_PRIO       			8 
//设置任务堆栈大小
#define Receive_STK_SIZE  		    		64
//任务堆栈
OS_STK Receive_TASK_STK[Receive_STK_SIZE];
//任务函数
void  Receive_task(void *pdata);





//主任务
//设置任务优先级
 #define MAIN_TASK_PRIO 4 
//设置任务堆栈大小
#define MAIN_STK_SIZE  					128
//任务堆栈	
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//任务函数
void main_task(void *pdata);

//信号量集任务
//设置任务优先级

//设置任务堆栈大小
#define MASTER_STK_SIZE  		 		64
//任务堆栈	
OS_STK MASTER_TASK_STK[MASTER_STK_SIZE];
//任务函数
 void master_task(void *pdata);
 



//按键扫描任务
//设置任务优先级
#define KEY_TASK_PRIO       			2 
//设置任务堆栈大小
#define KEY_STK_SIZE  					64
//任务堆栈	
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
//任务函数
void key_task(void *pdata);


  	   


extern box mybox;
  
extern  u16  dog_clock;

extern OS_EVENT * RS485_MBOX;			//	rs485邮箱信号量

extern OS_EVENT *Heartbeat;			 //心跳信号量

extern u8 cont;//用于更改主机号的记次数器
extern  u8 token[33];//主机号令牌


//接收缓存区 	

//////////////////////////////////////////



int subcontrol(u8,u8);

#define ID  6
/////////////////////////////////////////////
int main(void)
 {	 
  
	 
	
	delay_init();	    	 //延时函数初始化	  
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
 	LED_Init();			     //LED端口初始化

	RS485_Init(9600);	//初始化RS485
	TIM3_Int_Init(4999,7199);//10Khz的计数频率，计数5K次为500ms  
	 initmybox(ID);
	OSInit();  	 			//初始化UCOSII
			  
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	    
}							    
//开始任务
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;  	    
	pdata = pdata; 	
	 Heartbeat=OSSemCreate(0);
	 RS485_MBOX=OSMboxCreate((void*)0);
	OSStatInit();					//初始化统计任务.这里会延时1秒钟左右	
 	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    			   
 	OSTaskCreate(master_task,(void *)0,(OS_STK*)&MASTER_TASK_STK[MASTER_STK_SIZE-1],MASTER_TASK_PRIO);	 				   
 	OSTaskCreate(Heartbeat_task,(void *)0,(OS_STK*)&TAKE_TASK_STK[TAKE_STK_SIZE-1],TAKE_TASK_PRIO);
	OSTaskCreate(Receive_task,(void *)0,(OS_STK*)&Receive_TASK_STK[Receive_STK_SIZE-1],Receive_TASK_PRIO);
 	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}
//LED任务


void Receive_task(void *pdate)//接收任务
{   u8 err;
	 u8 *msg;
	 int flag1,flag2;
	
    while(1)
    	{
		 if(mybox.master==1)OSTaskSuspend(Receive_TASK_PRIO);
	 msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//接收到有数据
	 flag1=rs485_trans_order(msg);
	 	 
	 if(flag1==0);/***主机不是给本机通信，信息舍弃***/
	 if(flag1==1)/***是本机信息***/
	 	{		LED1=!LED1;	  
		       dog_clock=10;	 
	 	      flag2=subcontrol(mybox.relay,mybox.message);
		       if(flag2==0);/******心跳信息，主机活着，不必启动从新定位主机函数，返回给主机本从机也存在    喂狗*********/
               if(flag2==1);/*****下位机，控制命令，喂狗*****/		  
	 	}

	/* 时间到达函数,判断是否主机已经死亡*/

	
	/*	if(key)//接收到有数据
		{
			if(key>5)key=5;//最大是5个数据.
 			for(i=0;i<key;i++)LCD_ShowxNum(60+i*32,230,rs485buf[i],3,16,0X80);	//显示数据
 		}
		t++; 
		delay_ms(10);
		if(t==20)
		{
			LED0=!LED0;//提示系统正在运行	
			t=0;
			cnt++;
			LCD_ShowxNum(60+48,150,cnt,3,16,0X80);	//显示数据
		}
    	}*/
	}
	}
 /**************主机任务**********************/
  void master_task(void *pdata)
  {	  OS_CPU_SR cpu_sr=0;
      u8 go=1,i;
   while(1){
  	if(mybox.master==1)
	{	if(go==0)
	  { OSSemPost(Heartbeat);
			delay_ms(100);
			
      }
		if(go==1)
		{
		   	for(i=1;i<33;i++)
		{	   LED0=!LED0;
	       order_trans_rs485(mybox.myid,i,1,1,1);
		   delay_us(20000);
		   order_trans_rs485(mybox.myid,i,1,1,0);
		   delay_us(20000);
		  order_trans_rs485(mybox.myid,i,1,2,1);
		  delay_us(20000);
		   order_trans_rs485(mybox.myid,i,1,2,0);
		    delay_us(50000);
		}	
		}
		                                      //启动接收程序
	}
	if(mybox.master==0)
	{
		OS_ENTER_CRITICAL();
		 OSTaskResume(Receive_TASK_PRIO );//启动从机任务状态
		OSTaskSuspend(MASTER_TASK_PRIO );//挂起主机任状态.
		OS_EXIT_CRITICAL();	
	}
	}//while
  }



 /***********************************/

int subcontrol(u8 i,u8 j)//给下下位机放指令	 
{
 if(mybox.send==0)//心态脉搏
   	{

	return 0;
    }
   if(mybox.send==1) //下位机控制
   	{ 
   	if(i==1&&j==1)GPIO_ResetBits(GPIOA,GPIO_Pin_0);
    if(i==1&&j==0)GPIO_SetBits(GPIOA,GPIO_Pin_0);
    if(i==2&&j==1)GPIO_ResetBits(GPIOA,GPIO_Pin_8);
    if(i==2&&j==0)GPIO_SetBits(GPIOA,GPIO_Pin_8);
	return 1;
   	}
if(mybox.send==2)//修改主机记录数组
 {
modfiy_token_array(mybox.relay,mybox.message);
return 2;
 }
return 3; //操作失败
}



 

