
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
#include "lcd.h"//������
/////////////////////////UCOSII��������///////////////////////////////////
//START ����
//�����������ȼ�
#define START_TASK_PRIO      			10 //��ʼ��������ȼ�����Ϊ���
//���������ջ��С
#define START_STK_SIZE  				64
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);	
 			   
//LED����
//�����������ȼ�
#define TAKE_TASK_PRIO       			7 
//���������ջ��С
#define TAKE_STK_SIZE  		    		64
//�����ջ
OS_STK TAKE_TASK_STK[TAKE_STK_SIZE];
//������
void Heartbeat_task(void *pdata);

//��������
//�����������ȼ�
#define Receive_TASK_PRIO       			8 
//���������ջ��С
#define Receive_STK_SIZE  		    		64
//�����ջ
OS_STK Receive_TASK_STK[Receive_STK_SIZE];
//������
void  Receive_task(void *pdata);





//������
//�����������ȼ�
 #define MAIN_TASK_PRIO 4 
//���������ջ��С
#define MAIN_STK_SIZE  					128
//�����ջ	
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//������
void main_task(void *pdata);

//�ź���������
//�����������ȼ�

//���������ջ��С
#define MASTER_STK_SIZE  		 		64
//�����ջ	
OS_STK MASTER_TASK_STK[MASTER_STK_SIZE];
//������
 void master_task(void *pdata);
 

#define MYLED_TASK_PRIO       			13 
//���������ջ��С
#define MYLED_STK_SIZE  		    		64
//�����ջ
OS_STK MYLED_TASK_STK[MYLED_STK_SIZE];
//������
void myled_task(void *pdata);
  	   


extern box mybox;
  
extern  u16  dog_clock;

extern OS_EVENT * RS485_MBOX,*RS485_STUTAS_MBOX;			//	rs485�����ź���

extern OS_EVENT *Heartbeat;			 //�����ź���

extern u8 cont;//���ڸ��������ŵļǴ�����
extern  u8 token[33];//����������

extern status_box mystatus;

extern status_list_node system_status_list_1[33];

extern status_list_node system_status_list_2[33];

extern u8 hguestnum;

//���ջ�����

//////////////////////////////////////////

u8 led_lock=0;

int subcontrol(u8,u8);

#define ID  8

#define SIZE_1 20
#define SIZE_2 20
#define WORK_STATUS_1	 0//0Ϊû�й���  1Ϊ����  2Ϊ��������ʼ��Ϊ0
#define WORK_STATUS_2    0 
#define WORK_TIME_1 0
#define WORK_TIME_2	0
/////////////////////////////////////////////
int main(void)
 {	 
  
	 
	
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
 	LED_Init();			     //LED�˿ڳ�ʼ��
/*************************/
	HT1621_Init();
	delay_us(100000);
	KEY_Init();          //��ʼ���밴�����ӵ�Ӳ���ӿ�  
	AT24CXX_Init();			//IIC��ʼ��
	Adc_Init();
/************************************/

		uart_init(9600);LCD_Init();	                                                              //������ʾ
	RS485_Init(9600);	//��ʼ��RS485
	TIM4_Int_Init(4999,7199);//10Khz�ļ���Ƶ�ʣ�����5K��Ϊ500ms 
//	TIM3_Cap_Init(0XFFFF,72-1);	//��1Mhz��Ƶ�ʼ���	  //��������LCDʱ������ע���˾٣����ų�ͻ
	 initmybox(ID);
	 init_mystatus(ID,SIZE_1,SIZE_2,WORK_STATUS_1,WORK_STATUS_2,WORK_TIME_1,WORK_TIME_2);
	 
	OSInit();  	 			//��ʼ��UCOSII
			  
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();	    
}							    
//��ʼ����
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;  	    
	pdata = pdata; 	
	 Heartbeat=OSSemCreate(0);
	 RS485_MBOX=OSMboxCreate((void*)0);
	 RS485_STUTAS_MBOX=OSMboxCreate((void*)0);
	OSStatInit();					//��ʼ��ͳ������.�������ʱ1��������	
 	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    			   
 	OSTaskCreate(master_task,(void *)0,(OS_STK*)&MASTER_TASK_STK[MASTER_STK_SIZE-1],MASTER_TASK_PRIO);	 				   
 	OSTaskCreate(Heartbeat_task,(void *)0,(OS_STK*)&TAKE_TASK_STK[TAKE_STK_SIZE-1],TAKE_TASK_PRIO);
	OSTaskCreate(Receive_task,(void *)0,(OS_STK*)&Receive_TASK_STK[Receive_STK_SIZE-1],Receive_TASK_PRIO);
	OSTaskCreate(myled_task,(void *)0,(OS_STK*)&MYLED_TASK_STK[MYLED_STK_SIZE-1],MYLED_TASK_PRIO);
 	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
}
//LED����

 /**************�ӻ�����**********************/
void Receive_task(void *pdate)//�ӻ�����
{   u8 err;
	 u8 *msg;
	 int flag1,flag2;
	
    while(1)
    	{
		 if(mybox.master==1)OSTaskSuspend(Receive_TASK_PRIO);
	 msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//���յ�������
	 flag1=rs485_trans_order(msg);
	 dog_clock=10;	 
	 if(flag1==0);/***�������Ǹ�����ͨ�ţ���Ϣ����***/
	 if(flag1==1)/***�Ǳ�����Ϣ***/
	 	{		//LED1=!LED1;	  
		       dog_clock=10;	 
	 	      flag2=subcontrol(mybox.relay,mybox.message);
		       if(flag2==0);/******������Ϣ���������ţ������������¶�λ�������������ظ��������ӻ�Ҳ����    ι��*********/
               if(flag2==1);/*****��λ�����������ι��*****/		  
	 	}

      if(led_lock==0)key_idset();
	}
	}
 /**************��������**********************/
  void master_task(void *pdata)	  //��������
  {	  OS_CPU_SR cpu_sr=0;
      u8 go=2,i;
	  // u8 *msg,err;
	  u8 try_cont=0;
	  for(i=1;i<33;i++){set_statuslist_1(i,0,0,0);set_statuslist_2(i,0,0,0);}//��ʼ������״̬����
   while(1)
   	{
  	if(mybox.master==1)
     {	hguestnum=111;
	   if(go==0)
	  { myled(); 
	//    LED0=!LED0;
	    OSSemPost(Heartbeat);
		  delay_ms(100);
			
      }
		if(go==1)
		{
		{OSSemPost(Heartbeat);delay_ms(100);try_cont++;}  //����ʱ��װ�ɺ���
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
                        for(i=1;i<33;i++) //�رմӻ�led���ܣ���ֹͨ�ſ�����Ϣ�Ķ�ʧ
                        	{led_on_off(0);}
			   for(i=1;i<33;i++)		 //�ռ��ӻ�״̬
               { inquiry_slave_status(i);	
	         }
			   //��������LCD�����Ի�����Ϣ����
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
				   if(system_status_list_1[i].work_status==0){order_trans_rs485(mybox.myid,i,1,1,1); LED0=!LED0;}//��֤�ӻ�9�Ŀ��ƺ͹�����ʱ
				   if(system_status_list_2[i].work_status==0){order_trans_rs485(mybox.myid,i,1,2,1);}//
				 	}
		       }
			 	if(go==3){// LED0=!LED0;
				  order_trans_rs485(mybox.myid,2,2,0,0);
				  delay_us(10000);
				  }
				if(go==4){ ;}
		
		}
		                                      //�������ճ�
	if(mybox.master==0)
	{
		OS_ENTER_CRITICAL();
		 OSTaskResume(Receive_TASK_PRIO );//�����ӻ�����״̬
		OSTaskSuspend(MASTER_TASK_PRIO );//����������״̬.
		OS_EXIT_CRITICAL();	
	}
	}//while
 	}

void myled_task(void *pdata)
{
while(1)
{
if(mybox.master==1)OSTaskSuspend( MYLED_TASK_PRIO);
if(led_lock==0)key_idset();
delay_ms(100);
}
}
 /***********************************/

int subcontrol(u8 i,u8 j)//������λ����ָ��	 
{
 if(mybox.send==0)//��̬����
   	{
	   //LED1=!LED1;
	return 0;
    }
   if(mybox.send==1) //��λ������
   	{ 	 // LED1=!LED1;
   	if(i==1&&j==1){GPIO_ResetBits(GPIOA,GPIO_Pin_0);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);LED0=!LED0; }
    if(i==1&&j==0){GPIO_SetBits(GPIOA,GPIO_Pin_0);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);}
    if(i==2&&j==1){GPIO_ResetBits(GPIOA,GPIO_Pin_8);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);LED0=!LED0; }
    if(i==2&&j==0){GPIO_SetBits(GPIOA,GPIO_Pin_8);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);}
	return 1;
   	}
if(mybox.send==2)//�鿴�ӻ�״̬
 {
 	LED1=!LED1;
  status_trans_rs485(&mystatus);
return 2;
 }
 if(mybox.send==3) //�ر�ˢ��led��Ļ
 {
 led_lock=1;
 return 3;
 }
 if(mybox.send==4) //��ˢ��led��Ļ
 {
  led_lock=0;
 return 4;
 }
return 5; //����ʧ��
}




 
