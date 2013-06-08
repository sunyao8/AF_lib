
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
//主任务
//设置任务优先级
 #define MAIN_TASK_PRIO                              1               //主从机都有的任务 
//设置任务堆栈大小
#define MAIN_STK_SIZE  					128
//任务堆栈	
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//任务函数
void main_task(void *pdata);
//////////////////////////////////////////////////////////////////////////

#define SETID_TASK_PRIO       			2 
//设置任务堆栈大小
#define SETID_STK_SIZE  		    		64
//任务堆栈
OS_STK SETID_TASK_STK[SETID_STK_SIZE];
//任务函数
void SETID_task(void *pdata);
//////////////////////////////////////////////////////////////////////////////
#define 	M1_OPEN_TASK_PRIO       	       3 
//设置任务堆栈大小
#define  M1_OPEN_STK_SIZE  		    		32
//任务堆栈
OS_STK M1_OPEN_TASK_STK[M1_OPEN_STK_SIZE];
//任务函数
void sub_machine1_open_task(void *pdata);

#define M2_OPEN_TASK_PRIO       			4 
//设置任务堆栈大小
#define M2_OPEN_STK_SIZE  		    		32
//任务堆栈
OS_STK M2_OPEN_TASK_STK[M2_OPEN_STK_SIZE];
//任务函数
void sub_machine2_open_task(void *pdata);

#define M1_CLOSE_TASK_PRIO       			5 
//设置任务堆栈大小
#define M1_CLOSE_STK_SIZE  		    		32
//任务堆栈
OS_STK M1_CLOSE_TASK_STK[M1_CLOSE_STK_SIZE];
//任务函数
void sub_machine1_close_task(void *pdata);


#define M2_CLOSE_TASK_PRIO       			6 
//设置任务堆栈大小
#define M2_CLOSE_STK_SIZE  		    		32
//任务堆栈
OS_STK M2_CLOSE_TASK_STK[M2_CLOSE_STK_SIZE];
//任务函数
void sub_machine2_close_task(void *pdata);

/////////////////////////////////////////////////////////////////////////////
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
#define Receive_TASK_PRIO       			8 
//设置任务堆栈大小
#define Receive_STK_SIZE  		    		64
//任务堆栈
OS_STK Receive_TASK_STK[Receive_STK_SIZE];
//任务函数
void  Receive_task(void *pdata);
////////////////////////////////////////////////////////////////////////

#define SCAN_TASK_PRIO       			9 
//设置任务堆栈大小
#define SCAN_STK_SIZE  		    		64
//任务堆栈
OS_STK SCAN_TASK_STK[SCAN_STK_SIZE];
//任务函数
void scanf_task(void *pdata);


////////////////////////////////////////////////////////////////////////

#define MASTERLED_TASK_PRIO       			10 
//设置任务堆栈大小
#define MASTERLED_STK_SIZE  		    		128
//任务堆栈
OS_STK MASTERLED_TASK_STK[MASTERLED_STK_SIZE];
//任务函数
void masterled_task(void *pdata);

////////////////////////////////////////////////////////////////////////
#define MYLED_TASK_PRIO       			11 
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

extern OS_EVENT * RS485_MBOX,*RS485_STUTAS_MBOX;			//	rs485邮箱信号量

extern OS_EVENT *Heartbeat;			 //心跳信号量
extern OS_EVENT *master_led_task;

extern OS_EVENT * sub_machine1_open;		//下位机命令信号
extern OS_EVENT * sub_machine1_close;		//下位机命令信号
extern OS_EVENT * sub_machine2_open;		//下位机命令信号
extern OS_EVENT * sub_machine2_close;		//下位机命令信号

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

//接收缓存区

//////////////////////////////////////////


u8 led_lock=0;
u8 init=1;

int slave_control(u8,u8);

//#define ID  1

#define SIZE_1 100
#define SIZE_2 200
#define WORK_STATUS_1	 0//0为没有工作  1为工作  2为坏掉，初始化为0
#define WORK_STATUS_2    0 
#define WORK_TIME_1 0
#define WORK_TIME_2	0
/////////////////////////////////////////////
int main(void)
 {	 
  
	 
	
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
	TIM4_Int_Init(9999,7199);//10Khz的计数频率，计数10K次为1000ms 
	TIM3_Cap_Init(0XFFFF,72-1);	//以1Mhz的频率计数	  //开发板用LCD时，必须注掉此举，引脚冲突
	 initmybox();
	 init_mystatus(SIZE_1,SIZE_2,WORK_STATUS_1,WORK_STATUS_2,WORK_TIME_1,WORK_TIME_2);
	 
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
	 sub_machine1_close=OSSemCreate(0);	//创建信号量
	 sub_machine1_open=OSSemCreate(0);	//创建信号量
	 sub_machine2_close=OSSemCreate(0);	//创建信号量
	 sub_machine2_open=OSSemCreate(0);	//创建信号量
	 scan_slave=OSSemCreate(0);
	OSStatInit();					//初始化统计任务.这里会延时1秒钟左右	
 	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    			   
 	OSTaskCreate(master_task,(void *)0,(OS_STK*)&MASTER_TASK_STK[MASTER_STK_SIZE-1],MASTER_TASK_PRIO);	 				   
	OSTaskCreate(Receive_task,(void *)0,(OS_STK*)&Receive_TASK_STK[Receive_STK_SIZE-1],Receive_TASK_PRIO);
	OSTaskCreate(myled_task,(void *)0,(OS_STK*)&MYLED_TASK_STK[MYLED_STK_SIZE-1],MYLED_TASK_PRIO);
	OSTaskCreate(masterled_task,(void *)0,(OS_STK*)&MASTERLED_TASK_STK[MASTERLED_STK_SIZE-1],MASTERLED_TASK_PRIO);
	OSTaskCreate(SETID_task,(void *)0,(OS_STK*)&SETID_TASK_STK[SETID_STK_SIZE-1],SETID_TASK_PRIO);
	OSTaskCreate(sub_machine1_open_task,(void *)0,(OS_STK*)&M1_OPEN_TASK_STK[M1_OPEN_STK_SIZE-1],M1_OPEN_TASK_PRIO);
	OSTaskCreate(sub_machine1_close_task,(void *)0,(OS_STK*)&M1_CLOSE_TASK_STK[M1_CLOSE_STK_SIZE-1],M1_CLOSE_TASK_PRIO);
	OSTaskCreate(sub_machine2_open_task,(void *)0,(OS_STK*)&M2_OPEN_TASK_STK[M2_OPEN_STK_SIZE-1],M2_OPEN_TASK_PRIO);
	OSTaskCreate(sub_machine2_close_task,(void *)0,(OS_STK*)&M2_CLOSE_TASK_STK[M2_CLOSE_STK_SIZE-1],M2_CLOSE_TASK_PRIO);
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
                     OSTaskResume(MASTER_TASK_PRIO );//启动主机任务状态
			OSTaskResume(MASTERLED_TASK_PRIO );//启动主机任务状态
			OSTaskSuspend(Receive_TASK_PRIO);//挂起从机任务

		        }
	 msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//接收到有数据
	 flag1=rs485_trans_order(msg);
	 dog_clock=10;
	 if(flag1==1)/***是本机信息***/
	 	{		//LED1=!LED1;	  
		       dog_clock=10;
	 	      slave_control(mybox.relay,mybox.message);
			  	   
	 	}

      if(led_lock==0){temperature();key_lcd();}
	}
	}
 /**************主机任务**********************/
  void master_task(void *pdata)	  //主机任务
  {	  OS_CPU_SR cpu_sr=0;
      u8 go=0;
	  u32 i;
	  // u8 *msg,err;
	  u8 try_cont=0;
   while(1)
   	{
  	if(mybox.master==1)
     {	hguestnum=111;
	if(init==1)
		{
	//		dianya_zhi=400;//防止报警测试使用
	//		tempshuzhi=30;//防止报警测试使用
    //   delay_time(5000);//初始化延时，使系统稳定
    	  led_on_off(ALL_NODE_LCD_LOCK,0);
	   scanf_slave_machine();
	for(i=1;i<=slave[0];i++){set_statuslist_1(slave[i],0,0,0);set_statuslist_2(slave[i],0,0,0);}//初始化两个状态队�  
		  	   init=0;
		}
	   if(go==0)
	  { myled(); 
	//    LED0=!LED0;
	   delay_time(1);
		 if(((gonglvshishu)<90)&&(L_C_flag==1))
		 	{
		 	offset_idlepower();
		       }
		 if((gonglvshishu)>95&&(L_C_flag==1))
		 	{
                        unload_power(system_status_list_1,system_status_list_2);
		       }
	        if((idle_time>400)&&(gonglvshishu>90&&gonglvshishu<95)&&(L_C_flag==1))
                {idle_time=0;turn_power(system_status_list_1,system_status_list_2);}
		if(L_C_flag==0){C_unload_power(system_status_list_1, system_status_list_2);}
		if(scan_time>120*second){OSSemPost(scan_slave);scan_time=0;}
	   }
		if(go==1)
		{
		{OSSemPost(Heartbeat);delay_ms(100);try_cont++;}  //把延时封装成函数
		   	if(try_cont==20)
		  for(i=1;i<33;i++)
		  {	//LED0=!LED0;  
	       order_trans_rs485(mybox.myid,i,1,1,1);
		   delay_us(20000);
		  	order_trans_rs485(mybox.myid,i,1,2,1); 
		   delay_us(20000);
		   }
		   if(try_cont==40)
		   for(i=1;i<33;i++)
		   {
			// LED0=!LED0; 
		   order_trans_rs485(mybox.myid,i,1,1,0);
		  delay_us(20000);
		   order_trans_rs485(mybox.myid,i,1,2,0);
		    delay_us(50000);
			try_cont=0;
			}
		}	
		if(go==2)
			   {
                 myled(); 
				                   
                        	led_on_off(0,0);		 //关闭从机led功能，防止通信控制信息的丢失
			   for(i=1;i<33;i++)		 //收集从机状态
               { inquiry_slave_status(i);	
	         }
			   //下面是用LCD屏测试回馈信息功能
				 for(i=1;i<33;i++)
				   {
				   LCD_ShowxNum(5+1*25,i*15,system_status_list_1[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+2*25,i*15,system_status_list_1[i].size,3,16,0X80);
				   LCD_ShowxNum(5+3*25,i*15,system_status_list_1[i].work_status,3,16,0X80);	
			       LCD_ShowxNum(5+4*25,i*15,system_status_list_1[i].work_time,3,16,0X80);
				   LCD_ShowxNum(5+5*25,i*15,system_status_list_2[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+6*25,i*15,system_status_list_2[i].size,3,16,0X80);
				   LCD_ShowxNum(5+7*25,i*15,system_status_list_2[i].work_status,3,16,0X80);	
			       LCD_ShowxNum(5+8*25,i*15,system_status_list_2[i].work_time,3,16,0X80);
				   }
			 	
				 for(i=1;i<33;i++)
					{
				   if(system_status_list_1[i].work_status==0){order_trans_rs485(mybox.myid,i,1,1,1);  delay_us(10000);}//验证从机9的控制和工作计时
				   if(system_status_list_2[i].work_status==0){order_trans_rs485(mybox.myid,i,1,2,1);  delay_us(10000);}//				     
				 	}
                       
				
				     //打开从机led功能
                        	led_on_off(1,0);
		       }
			 	if(go==3){// LED0=!LED0;
				  order_trans_rs485(mybox.myid,2,2,0,0);
				  delay_us(10000);
				  }
				
				if(go==4){ 
					s8 c=0,d=0;
			dianya_zhi=400;//防止报警
			tempshuzhi=30;
                  //     myled() ; 
                  
			
				   LCD_ShowxNum(5+1*25,10*15,slave[0],3,16,0X80);

							 for(i=1;i<=slave[0];i++)
					{
			   if(system_status_list_1[slave[i]].work_status==0){led_on_off(NODE_LCD_LOCK_BASE,slave[i]);order_trans_rs485(mybox.myid,slave[i],Sub_Order,1,1);  delay_us(10000);try_cont++;}//验证从机9的控制和工作计时
                                             					
							 }
							 
											for(i=1;i<=slave[0];i++)
										   if(system_status_list_2[slave[i]].work_status==0){led_on_off(NODE_LCD_LOCK_BASE,slave[i]);order_trans_rs485(mybox.myid,slave[i],Sub_Order,2,1);  delay_us(10000);try_cont++;}//				     

           			led_on_off(BUSY_NODE_LCD_LCOK,0);		 //关闭从机led功能，防止通信控制信息的丢失                                      
			   for(i=1;i<=slave[0];i++)		 //收集从机状态
               { inquiry_slave_status(slave[i]);	
	         }
                             c=sort_busynode_list_asc(sort_busy_list_2,system_status_list_2);
                             d=sort_busynode_list_asc(sort_busy_list_1, system_status_list_1);
			   //下面是用LCD屏测试回馈信息功能
				 for(i=1;i<=d;i++)
				   {
				   LCD_ShowxNum(5+1*25,i*15,sort_busy_list_1[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+2*25,i*15,sort_busy_list_1[i].size,3,16,0X80);
				   LCD_ShowxNum(5+3*25,i*15,sort_busy_list_1[i].work_time,3,16,0X80);	
  
					}
					for(i=1;i<=c;i++)
					{
				   LCD_ShowxNum(5+6*25,i*15,sort_busy_list_2[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+7*25,i*15,sort_busy_list_2[i].size,3,16,0X80);
				   LCD_ShowxNum(5+8*25,i*15,sort_busy_list_2[i].work_time,3,16,0X80);	
			       
						}
                        	
		 

					}

		if(go==5)
			{

                                                      s8 d=0,c=0;
								  led_on_off(1,0);
						   for(i=1;i<33;i++)		 //收集从机状态
               { inquiry_slave_status(i);	
	         }			  
					
		c=sort_idlenode_list(sort_idle_list_1, system_status_list_1);
		 d=sort_idlenode_list(sort_idle_list_2,system_status_list_2);			 

				    for(i=1;i<=d;i++)
				   {
                                  LCD_ShowxNum(5+1*25,i*15,sort_idle_list_2[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+2*25,i*15,sort_idle_list_2[i].size,3,16,0X80);

				   }
				for(i=1;i<=c;i++)
					{                        LCD_ShowxNum(5+6*25,i*15,sort_idle_list_1[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+7*25,i*15,sort_idle_list_1[i].size,3,16,0X80);

					}
				 led_on_off(1,0);	
		}

     if(go==6)
     	{
          	while(1)
	{
               delay_time(1);
		   myled(); 


	  
	 
	}	 

	 }
if(go==7)
{
u8 DAT[10]={
			0x7f,0xbf,0xdf,0xef,0xf7,0xfb,0xfd,0xfe,0xff
			};

	while(1)
	{
	  
	 for(i=0;i<9;i++)
	 {HT595_Send_Byte(DAT[i]);
	 delay_ms(40000);
	 delay_ms(40000);
	 }

		}

}

	}
		                                      //启动接收程
	if(mybox.master==0)
	{
		OS_ENTER_CRITICAL();
		 OSTaskResume(Receive_TASK_PRIO );//启动从机任务状态
		OSTaskSuspend(MASTER_TASK_PRIO );//挂起主机任状态.
		OS_EXIT_CRITICAL();	
	}
	}//while
 	}

void myled_task(void *pdata)
{
while(1)
{
if(mybox.master==1)OSTaskSuspend( MYLED_TASK_PRIO);
 if(led_lock==0){temperature();key_lcd();}
delay_ms(50);
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
                                   OSTaskSuspend(MASTERLED_TASK_PRIO);
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
               OSTaskResume(MASTERLED_TASK_PRIO);			   
               OSTaskSuspend(SETID_TASK_PRIO);
			   OS_EXIT_CRITICAL();	
			 }

		  }

}





void masterled_task(void *pdata)
{
while(1)
{
if(mybox.master==0)OSTaskSuspend( MASTERLED_TASK_PRIO);
gonglvyinshu();//计算功率，电压电流与显示按键分开
temperature();
key_lcd();
delay_time(1);
delay_ms(50);//没有延时，屏会死机
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
   if(mybox.send==1) //下位机控制
   	{ 	 // LED1=!LED1;
   	if(i==1&&j==1){GPIO_ResetBits(GPIOA,GPIO_Pin_0);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);LED0=!LED0; }
    if(i==1&&j==0){GPIO_SetBits(GPIOA,GPIO_Pin_0);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);}
    if(i==2&&j==1){GPIO_ResetBits(GPIOA,GPIO_Pin_8);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);LED0=!LED0; }
    if(i==2&&j==0){GPIO_SetBits(GPIOA,GPIO_Pin_8);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);}
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	return 1;
   	}
if(mybox.send==2)//查看从机状态
 {
 //	LED1=!LED1;
  status_trans_rs485(&mystatus);
 led_lock=0;//操作完成开锁
return 2;
 }
 if(mybox.send==ALL_NODE_LCD_UNLOCK) //打开刷新led屏幕
 {
 led_lock=0;
 return 3;
 }
 if(mybox.send==ALL_NODE_LCD_LOCK) //关闭刷新led屏幕
 {
  led_lock=1;
 return 4;
 }
  if(mybox.send==IDLE_NODE_LCD_LOCK) //如果有空闲电容器关闭刷新led屏幕
 {
 if(mystatus.work_status[0]==0||mystatus.work_status[1]==0){led_lock=1;return 4;}
 }
 if(mybox.send==BUSY_NODE_LCD_LCOK) //如果有忙碌电容器关闭刷新led屏幕
 {
 if(mystatus.work_status[0]==1||mystatus.work_status[1]==1){led_lock=1;return 4;}
 }
if((mybox.send-mybox.myid)==NODE_LCD_LOCK_BASE)
{
led_lock=1;
return 4;
}
if(mybox.send>=Sub_Order&&mybox.send<=(Sub_Order+64))
{
    if(i==1&&j==1)
{
//led_lock=0;//操作完成开锁
OSSemPost(sub_machine1_close);
}
    if(i==1&&j==0)
{
//led_lock=0;//操作完成开锁
OSSemPost(sub_machine1_open);
}
    if(i==2&&j==1)
{
//led_lock=0;//操作完成开锁
OSSemPost(sub_machine2_close);
}
    if(i==2&&j==0)
{
//led_lock=0;//操作完成开锁
OSSemPost(sub_machine2_open);
}												
return 5;
}


return 6; //操作失败
}

void sub_machine1_close_task(void *pdate)//从机任务
{   u8 err;
    // u16 time=0;
    while(1)
    	{
        OSSemPend(sub_machine1_close,0,&err);
	set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);
//	if(m2_close==1)
//    	{ if(m2_closetime<15)
//    	         {  
//                  delay_ms(1000*(15-m2_closetime)+1);
//				time=1000*(15-m2_closetime)+1;  
//	          }
//
//	}
//	   
          sub_delaytime_15(mybox.send-Sub_Order);
			//	time=0;
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	true_worktime1_flag=1;
             LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
                  led_lock=0;//操作完成开锁

         }

}
void sub_machine1_open_task(void *pdate)//从机任务
{   u8 err;
    // u16 time=0;
    while(1)
    	{
        OSSemPend(sub_machine1_open,0,&err);
//	    if(m2_close==0)
//    	{ if(m2_opentime<5)
//    	         {
//                  delay_ms(1000*(5-m2_opentime)+1);
//				time=1000*(5-m2_opentime)+1;  
//	          }
//
//	}
          sub_delaytime_5(mybox.send-Sub_Order);
	//			  time=0;
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	true_worktime1_flag=0;
	LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	led_lock=0;//操作完成开锁
          {  
		 delay_ms(60000);
		 delay_ms(60000);		 
	set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);

         }
          	 	
	}

}


void sub_machine2_close_task(void *pdate)//从机任务
{   u8 err;
    while(1)
    	{
        OSSemPend(sub_machine2_close,0,&err);
set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);        
          sub_delaytime_15(mybox.send-Sub_Order);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	true_worktime2_flag=1;
        LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
		led_lock=0;//操作完成开锁

	}

}



void sub_machine2_open_task(void *pdate)//从机任务
{   u8 err;
    while(1)
    	{
        OSSemPend(sub_machine2_open,0,&err);
          sub_delaytime_5(mybox.send-Sub_Order);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	true_worktime2_flag=0;
	LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
           led_lock=0;//操作完成开锁
		{
		delay_ms(60000);
		delay_ms(60000);
	set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);				
	        }
	}

}

 void scanf_task(void *pdate)
 	{ u8 err;
while(1)
{
        OSSemPend(scan_slave,0,&err);
       led_on_off(IDLE_NODE_LCD_LOCK,0);
             scanf_slave_machine();
 

}
}
