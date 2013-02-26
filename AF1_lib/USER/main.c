
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
 



//����ɨ������
//�����������ȼ�
#define KEY_TASK_PRIO       			2 
//���������ջ��С
#define KEY_STK_SIZE  					64
//�����ջ	
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
//������
void key_task(void *pdata);


  	   


extern box mybox;
  
extern  u16  dog_clock;

extern OS_EVENT * RS485_MBOX;			//	rs485�����ź���

extern OS_EVENT *Heartbeat;			 //�����ź���

extern u8 cont;//���ڸ��������ŵļǴ�����
extern  u8 token[33];//����������


//���ջ����� 	

//////////////////////////////////////////



int subcontrol(u8,u8);

#define ID  6
/////////////////////////////////////////////
int main(void)
 {	 
  
	 
	
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
 	LED_Init();			     //LED�˿ڳ�ʼ��

	RS485_Init(9600);	//��ʼ��RS485
	TIM3_Int_Init(4999,7199);//10Khz�ļ���Ƶ�ʣ�����5K��Ϊ500ms  
	 initmybox(ID);
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
	OSStatInit();					//��ʼ��ͳ������.�������ʱ1��������	
 	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    			   
 	OSTaskCreate(master_task,(void *)0,(OS_STK*)&MASTER_TASK_STK[MASTER_STK_SIZE-1],MASTER_TASK_PRIO);	 				   
 	OSTaskCreate(Heartbeat_task,(void *)0,(OS_STK*)&TAKE_TASK_STK[TAKE_STK_SIZE-1],TAKE_TASK_PRIO);
	OSTaskCreate(Receive_task,(void *)0,(OS_STK*)&Receive_TASK_STK[Receive_STK_SIZE-1],Receive_TASK_PRIO);
 	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
}
//LED����


void Receive_task(void *pdate)//��������
{   u8 err;
	 u8 *msg;
	 int flag1,flag2;
	
    while(1)
    	{
		 if(mybox.master==1)OSTaskSuspend(Receive_TASK_PRIO);
	 msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//���յ�������
	 flag1=rs485_trans_order(msg);
	 	 
	 if(flag1==0);/***�������Ǹ�����ͨ�ţ���Ϣ����***/
	 if(flag1==1)/***�Ǳ�����Ϣ***/
	 	{		LED1=!LED1;	  
		       dog_clock=10;	 
	 	      flag2=subcontrol(mybox.relay,mybox.message);
		       if(flag2==0);/******������Ϣ���������ţ������������¶�λ�������������ظ��������ӻ�Ҳ����    ι��*********/
               if(flag2==1);/*****��λ�����������ι��*****/		  
	 	}

	/* ʱ�䵽�ﺯ��,�ж��Ƿ������Ѿ�����*/

	
	/*	if(key)//���յ�������
		{
			if(key>5)key=5;//�����5������.
 			for(i=0;i<key;i++)LCD_ShowxNum(60+i*32,230,rs485buf[i],3,16,0X80);	//��ʾ����
 		}
		t++; 
		delay_ms(10);
		if(t==20)
		{
			LED0=!LED0;//��ʾϵͳ��������	
			t=0;
			cnt++;
			LCD_ShowxNum(60+48,150,cnt,3,16,0X80);	//��ʾ����
		}
    	}*/
	}
	}
 /**************��������**********************/
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
		                                      //�������ճ���
	}
	if(mybox.master==0)
	{
		OS_ENTER_CRITICAL();
		 OSTaskResume(Receive_TASK_PRIO );//�����ӻ�����״̬
		OSTaskSuspend(MASTER_TASK_PRIO );//����������״̬.
		OS_EXIT_CRITICAL();	
	}
	}//while
  }



 /***********************************/

int subcontrol(u8 i,u8 j)//������λ����ָ��	 
{
 if(mybox.send==0)//��̬����
   	{

	return 0;
    }
   if(mybox.send==1) //��λ������
   	{ 
   	if(i==1&&j==1)GPIO_ResetBits(GPIOA,GPIO_Pin_0);
    if(i==1&&j==0)GPIO_SetBits(GPIOA,GPIO_Pin_0);
    if(i==2&&j==1)GPIO_ResetBits(GPIOA,GPIO_Pin_8);
    if(i==2&&j==0)GPIO_SetBits(GPIOA,GPIO_Pin_8);
	return 1;
   	}
if(mybox.send==2)//�޸�������¼����
 {
modfiy_token_array(mybox.relay,mybox.message);
return 2;
 }
return 3; //����ʧ��
}



 

