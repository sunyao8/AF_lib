
#include "rs485.h"
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "24cxx.h"
#include "includes.h" 	 
#include "adc.h"
#include "timer.h"
#include "485ANN.h"
#include "ht1621.h"
#include "key.h"
//32
#include "lcd.h"//测试用
/////////////////////////UCOSII任务设置///////////////////////////////////

#define SETID_TASK_PRIO       			1 
//设置任务堆栈大小
#define SETID_STK_SIZE  		    		64
//任务堆栈
OS_STK SETID_TASK_STK[SETID_STK_SIZE];
//任务函数
void SETID_task(void *pdata);
//////////////////////////////////////////////////////////////////////////////

//主机任务
//设置任务优先级

//设置任务堆栈大小
#define MASTER_STK_SIZE  		 		64
//任务堆栈	
OS_STK MASTER_TASK_STK[MASTER_STK_SIZE];
//任务函数
 void master_task(void *pdata);

////////////////////////////////////////////////////////////////////////////
//接收任务
//设置任务优先级
#define Receive_TASK_PRIO       			2 
//设置任务堆栈大小
#define Receive_STK_SIZE  		    		64
//任务堆栈
OS_STK Receive_TASK_STK[Receive_STK_SIZE];
//任务函数
void  Receive_task(void *pdata);
////////////////////////////////////////////////////////////////////////

#define SCAN_TASK_PRIO       			4 
//设置任务堆栈大小
#define SCAN_STK_SIZE  		    		64
//任务堆栈
OS_STK SCAN_TASK_STK[SCAN_STK_SIZE];
//任务函数
void scanf_task(void *pdata);


////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////
#define MYLED_TASK_PRIO       			5
//设置任务堆栈大小
#define MYLED_STK_SIZE  		    		128
//任务堆栈
OS_STK MYLED_TASK_STK[MYLED_STK_SIZE];
//任务函数
void myled_task(void *pdata);

/////////////////////////////////////////////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			12 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	
 			   
/////////////////////////////////////////////////////////////////////////

  	   
extern box mybox;
  
extern  u16  dog_clock;

extern OS_EVENT * RS485_MBOX,*RS485_STUTAS_MBOX,* RS485_RT;			//	rs485邮箱信号量

extern OS_EVENT *Heartbeat;			 //心跳信号量
extern OS_EVENT *master_led_task;


extern OS_EVENT *scan_slave;

extern u8 cont;//用于更改主机号的记次数器
extern  u8 token[33];//主机号令牌

extern status_box mystatus;

extern status_list_node system_status_list_1[33];

extern status_list_node system_status_list_2[33];

extern idle_list sort_idle_list_1[33];
extern idle_list sort_idle_list_2[33];
extern busy_list sort_busy_list_1[33];
extern busy_list sort_busy_list_2[33];

extern u16 dianya_zhi;
extern u8 hguestnum,gonglvshishu;
extern u32 idle_time,scan_time,dianliuzhi;
extern u16 wugongkvar;
extern s8 L_C_flag;
extern u8 id_num,tempshuzhi;
extern u8 slave[33];

extern u16 m1_opentime,m2_opentime,m1_closetime,m2_closetime;
extern u8 true_worktime1_flag,true_worktime2_flag;
extern u8 RT_FLAG;

//接收缓存区

//////////////////////////////////////////


u8 led_lock=0;
u8 init=20;
u8 auto_on=1;
int slave_control(u8,u8);
void EXTI_Configuration(void);//初始化函数

//#define ID  1

#define SIZE_1 10
#define SIZE_2 10
#define WORK_STATUS_1	 0//0为没有工作  1为工作  2为坏掉，初始化为0
#define WORK_STATUS_2    0 
#define WORK_TIME_1 0
#define WORK_TIME_2	0
/////////////////////////////////////////////
int main(void)
 {	 
  
	 
	u8 i;
	delay_init();	    	 //延时函数初始化	  
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
// 	LED_Init();			     //LED端口初始化
/*************************/
		delay_us(500000);
	HT1621_Init();
	KEY_Init();          //初始化与按键连接的硬件接口  
	AT24CXX_Init();			//IIC初始化
	Adc_Init();
/************************************/
///	uart_init(9600);LCD_Init();	                                                              //调试显示
	RS485_Init(9600);	//初始化RS485
	TIM4_Int_Init(9999*2,7199);//10Khz的计数频率，计数10K次为1000ms 
	 initmybox();
	 init_mystatus(SIZE_1,SIZE_2,WORK_STATUS_1,WORK_STATUS_2,WORK_TIME_1,WORK_TIME_2);
EXTI_Configuration();//初始化函数
for(i=0;i<30;i++)slave[i]=0;

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
	 RS485_STUTAS_MBOX=OSMboxCreate((void*)0);
	 scan_slave=OSSemCreate(0);
	 RS485_RT=OSMboxCreate((void*)0);
	OSStatInit();					//初始化统计任务.这里会延时1秒钟左右	
 	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    			   
	OSTaskCreate(master_task,(void *)0,(OS_STK*)&MASTER_TASK_STK[MASTER_STK_SIZE-1],MASTER_TASK_PRIO);	 				   
	OSTaskCreate(Receive_task,(void *)0,(OS_STK*)&Receive_TASK_STK[Receive_STK_SIZE-1],Receive_TASK_PRIO);
	OSTaskCreate(myled_task,(void *)0,(OS_STK*)&MYLED_TASK_STK[MYLED_STK_SIZE-1],MYLED_TASK_PRIO);
	OSTaskCreate(SETID_task,(void *)0,(OS_STK*)&SETID_TASK_STK[SETID_STK_SIZE-1],SETID_TASK_PRIO);
      OSTaskCreate(scanf_task,(void *)0,(OS_STK*)&SCAN_TASK_STK[SCAN_STK_SIZE-1],SCAN_TASK_PRIO);
 	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}
//LED任务

 /**************从机任务**********************/
void Receive_task(void *pdate)//从机任务
{   u8 err;
	 u8 *msg;
	 int flag1;
	
    while(1)
    	{
		 if(mybox.master==1)
		 	{
			OSTaskSuspend(Receive_TASK_PRIO);//挂起从机任务

		        }
	 msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//接收到有数据
	 flag1=rs485_trans_order(msg);
	 dog_clock=20;
mybox.myid=AT24CXX_ReadOneByte(0x0010);
	 if(flag1==1)/***是本机信息***/
	 	{		//LED1=!LED1;	  
		       dog_clock=20;
	 	      slave_control(mybox.relay,mybox.message);
			  	   
	 	}

	}
	}
 /**************主机任务**********************/
  void master_task(void *pdata)	  //主机任务
  {	  OS_CPU_SR cpu_sr=0;
	  // u8 *msg,err;
   while(1)
   	{
  	if(mybox.master==1)
     {	
     hguestnum=111;
	OSSemPost(scan_slave);
	
  if(init!=0) {init--;order_trans_rs485(mybox.myid,0,1,1,0,CONTROL);order_trans_rs485(mybox.myid,0,1,2,0,CONTROL);}
if(init==1)
{
RT_FLAG=0;
init=0;
}
	  computer_gonglu(system_status_list_1,system_status_list_2,slave);

	   delay_time(1);
	 
	 delay_ms(800);
  
			
			   
			     	

	}
		                                      //启动接收程
	if(mybox.master==0)
	{
		OS_ENTER_CRITICAL();
		OSTaskSuspend(MASTER_TASK_PRIO );//挂起主机任状态.
		OS_EXIT_CRITICAL();	
	}
	}//while
 	}

void myled_task(void *pdata)
{
while(1)
{
 	//temperature();
 key_idset();//按键与显示功能
  LIGHT(mystatus.work_status[0],mystatus.work_status[1]);//刷指示灯，如果显示器有旁路电容滤波 可以删除
        
delay_ms(100);
}
}




/********************************************/
void SETID_task(void *pdata)
{
        OS_CPU_SR cpu_sr=0;  	    	
          while(1)
          	{
		  id_num=AT24CXX_ReadOneByte(0x0010);
	///	  id_num=1;//测试开发板使用
		if(id_num<1||id_num>33)
			{            		
                                      mybox.master=2;
			             OS_ENTER_CRITICAL();
                      		OSTaskSuspend(MASTER_TASK_PRIO );//挂起主机任状态.
                      		OSTaskSuspend( MYLED_TASK_PRIO);
                                   OSTaskSuspend(Receive_TASK_PRIO);
						OS_EXIT_CRITICAL();
					HT595_Send_Byte(YELLOW_YELLOW|background_light_on);
                                      myled(); 									  
								   
		       }
               else if(id_num<=32&&id_num>=1)
               	{ 
                                  mybox.master=0;
				      mybox.myid=id_num;
				HT595_Send_Byte((GREEN_GREEN)|background_light_on);
				   OS_ENTER_CRITICAL();
		 OSTaskResume(MASTER_TASK_PRIO );//启动主机任务状态
		 OSTaskResume(MYLED_TASK_PRIO );//启动显示任务状态
		 OSTaskResume(Receive_TASK_PRIO );//启动从机任务状态
               OSTaskSuspend(SETID_TASK_PRIO);
			   OS_EXIT_CRITICAL();	
			 }

		  }

}








/******************************************/


 /***********************************/

int slave_control(u8 i,u8 j)//给下下位机放指令	 
{ 
 if(mybox.send==0)//心态脉搏
   	{
	   //LED1=!LED1;
	return 0;
    }
   if(mybox.send==1&&auto_on==1) //下位机控制
   	{ 	 // LED1=!LED1;
 if(i==1&&j==1)
 {GPIO_ResetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
 }
 if(i==1&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
 }
 if(i==2&&j==1)
 	{GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);

 }
 if(i==2&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);

 }
	return 1;
   	}
if(mybox.send==2)//查看从机状态
 {
  status_trans_rs485(&mystatus);
return 2;
 }
if(mybox.send==3&&auto_on==1)//查看从机状态
{
  status_trans_rs485_dis(&mystatus);
return 2;
 }
if(mybox.send==4&&auto_on==1)//初始投变比时使用，保证能投出去，带反馈机制
{
{ 	 // LED1=!LED1;
 if(i==1&&j==1)
 {GPIO_ResetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
 }
 if(i==1&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
 }
 if(i==2&&j==1)
 	{GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);

 }
 if(i==2&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);

 }
   	}
 status_trans_rs485_RT();//从机程序
return 2;
 }

 
if(mybox.send==5)//查看从机状态
 {
  status_trans_rs485_scantask(&mystatus);
return 5;
 }
if(mybox.send==6&&auto_on==1)//查看从机状态
 {
 { 	 // LED1=!LED1;
 if(i==1&&j==1)
 {GPIO_ResetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
 }
 if(i==1&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
 }
 if(i==2&&j==1)
 	{GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);

 }
 if(i==2&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);

 }
   	}
  status_trans_rs485_comm_RT();
	return 6;
 }

return 7; //操作失败
}

 void scanf_task(void *pdate)
 	{ u8 err;
while(1)
{
        OSSemPend(scan_slave,0,&err);
             scanf_slave_machine();
  init_Queue(system_status_list_1,slave,1);
  init_Queue(system_status_list_2,slave,2);


}
}


void EXTI_Configuration(void)//初始化函数

{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//打开时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE );	  //使能ADC1通道时钟

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	 		
	//使能外部中断复用时钟
	
	//映射GPIOE的Pin0至EXTILine0
GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);



EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);         	//嵌套分组为组0
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;      	//中断通道为通道10
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //抢断优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;    		//响应优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     		//开中断
	NVIC_Init(&NVIC_InitStructure);
	 EXTI_GenerateSWInterrupt(EXTI_Line12);

}
void EXTI15_10_IRQHandler(void)
{
static u8 first_sen=1;

	OSIntEnter();   

  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	
	{

if(KEY1==1&&auto_on==0)
 	{
		  auto_on=1;
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
GPIO_SetBits(GPIOA,GPIO_Pin_8);
mystatus.work_status[0]=0;
mystatus.work_status[1]=0;
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
 }

if(KEY1==0&&auto_on==1&&first_sen==1)	
 {
 		first_sen=2;
 auto_on=0;
GPIO_ResetBits(GPIOA,GPIO_Pin_0);
mystatus.work_status[0]=1;
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
 }
if(KEY1==0&&auto_on==1&&first_sen==2)
 	{
 	first_sen=1;		
 	auto_on=0;
GPIO_ResetBits(GPIOA,GPIO_Pin_8);
mystatus.work_status[1]=1;
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);

 }


	}
      EXTI_ClearITPendingBit(EXTI_Line12);

	   	OSIntExit();  

}


