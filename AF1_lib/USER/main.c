
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

#define SETID_TASK_PRIO       			1 
//���������ջ��С
#define SETID_STK_SIZE  		    		64
//�����ջ
OS_STK SETID_TASK_STK[SETID_STK_SIZE];
//������
void SETID_task(void *pdata);
//////////////////////////////////////////////////////////////////////////////

//��������
//�����������ȼ�

//���������ջ��С
#define MASTER_STK_SIZE  		 		64
//�����ջ	
OS_STK MASTER_TASK_STK[MASTER_STK_SIZE];
//������
 void master_task(void *pdata);

////////////////////////////////////////////////////////////////////////////
//��������
//�����������ȼ�
#define Receive_TASK_PRIO       			2 
//���������ջ��С
#define Receive_STK_SIZE  		    		64
//�����ջ
OS_STK Receive_TASK_STK[Receive_STK_SIZE];
//������
void  Receive_task(void *pdata);
////////////////////////////////////////////////////////////////////////

#define SCAN_TASK_PRIO       			9 
//���������ջ��С
#define SCAN_STK_SIZE  		    		64
//�����ջ
OS_STK SCAN_TASK_STK[SCAN_STK_SIZE];
//������
void scanf_task(void *pdata);


////////////////////////////////////////////////////////////////////////

#define MASTERLED_TASK_PRIO       			5 
//���������ջ��С
#define MASTERLED_STK_SIZE  		    		128
//�����ջ
OS_STK MASTERLED_TASK_STK[MASTERLED_STK_SIZE];
//������
void masterled_task(void *pdata);

////////////////////////////////////////////////////////////////////////
#define MYLED_TASK_PRIO       			4 
//���������ջ��С
#define MYLED_STK_SIZE  		    		128
//�����ջ
OS_STK MYLED_TASK_STK[MYLED_STK_SIZE];
//������
void myled_task(void *pdata);

/////////////////////////////////////////////////////////////////////////
//START ����
//�����������ȼ�
#define START_TASK_PRIO      			12 //��ʼ��������ȼ�����Ϊ���
//���������ջ��С
#define START_STK_SIZE  				64
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);	
 			   
/////////////////////////////////////////////////////////////////////////

  	   
extern box mybox;
  
extern  u16  dog_clock;

extern OS_EVENT * RS485_MBOX,*RS485_STUTAS_MBOX;			//	rs485�����ź���

extern OS_EVENT *Heartbeat;			 //�����ź���
extern OS_EVENT *master_led_task;


extern OS_EVENT *scan_slave;

extern u8 cont;//���ڸ��������ŵļǴ�����
extern  u8 token[33];//����������

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

//���ջ�����

//////////////////////////////////////////


u8 led_lock=0;
u8 init=1;
u8 auto_on=1;
int slave_control(u8,u8);
void EXTI_Configuration(void);//��ʼ������

//#define ID  1

#define SIZE_1 10
#define SIZE_2 10
#define WORK_STATUS_1	 0//0Ϊû�й���  1Ϊ����  2Ϊ��������ʼ��Ϊ0
#define WORK_STATUS_2    0 
#define WORK_TIME_1 0
#define WORK_TIME_2	0
/////////////////////////////////////////////
int main(void)
 {	 
  
	 
	
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
// 	LED_Init();			     //LED�˿ڳ�ʼ��
/*************************/
		delay_us(500000);
	HT1621_Init();
	KEY_Init();          //��ʼ���밴�����ӵ�Ӳ���ӿ�  
	AT24CXX_Init();			//IIC��ʼ��
	Adc_Init();
/************************************/
///	uart_init(9600);LCD_Init();	                                                              //������ʾ
	RS485_Init(9600);	//��ʼ��RS485
	TIM4_Int_Init(9999*2,7199);//10Khz�ļ���Ƶ�ʣ�����10K��Ϊ1000ms 
	 initmybox();
	 init_mystatus(SIZE_1,SIZE_2,WORK_STATUS_1,WORK_STATUS_2,WORK_TIME_1,WORK_TIME_2);
EXTI_Configuration();//��ʼ������

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
	 scan_slave=OSSemCreate(0);
	OSStatInit();					//��ʼ��ͳ������.�������ʱ1��������	
 	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    			   
	OSTaskCreate(master_task,(void *)0,(OS_STK*)&MASTER_TASK_STK[MASTER_STK_SIZE-1],MASTER_TASK_PRIO);	 				   
	OSTaskCreate(Receive_task,(void *)0,(OS_STK*)&Receive_TASK_STK[Receive_STK_SIZE-1],Receive_TASK_PRIO);
	OSTaskCreate(myled_task,(void *)0,(OS_STK*)&MYLED_TASK_STK[MYLED_STK_SIZE-1],MYLED_TASK_PRIO);
	OSTaskCreate(masterled_task,(void *)0,(OS_STK*)&MASTERLED_TASK_STK[MASTERLED_STK_SIZE-1],MASTERLED_TASK_PRIO);
	OSTaskCreate(SETID_task,(void *)0,(OS_STK*)&SETID_TASK_STK[SETID_STK_SIZE-1],SETID_TASK_PRIO);
      OSTaskCreate(scanf_task,(void *)0,(OS_STK*)&SCAN_TASK_STK[SCAN_STK_SIZE-1],SCAN_TASK_PRIO);
 	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
}
//LED����

 /**************�ӻ�����**********************/
void Receive_task(void *pdate)//�ӻ�����
{   u8 err;
	 u8 *msg;
	 int flag1;
	
    while(1)
    	{
		 if(mybox.master==1)
		 	{
                     OSTaskResume(MASTER_TASK_PRIO );//������������״̬
			OSTaskResume(MASTERLED_TASK_PRIO );//������������״̬
			OSTaskSuspend(Receive_TASK_PRIO);//����ӻ�����

		        }
	 msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//���յ�������
	 flag1=rs485_trans_order(msg);
	 dog_clock=20;
mybox.myid=AT24CXX_ReadOneByte(0x0010);
	 if(flag1==1)/***�Ǳ�����Ϣ***/
	 	{		//LED1=!LED1;	  
		       dog_clock=20;
	 	      slave_control(mybox.relay,mybox.message);
			  	   
	 	}

     // if(led_lock==0){temperature();key_lcd();}
	}
	}
 /**************��������**********************/
  void master_task(void *pdata)	  //��������
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
	//		dianya_zhi=400;//��ֹ��������ʹ��
	//		tempshuzhi=30;//��ֹ��������ʹ��
    //   delay_time(5000);//��ʼ����ʱ��ʹϵͳ�ȶ�
    	  led_on_off(ALL_NODE_LCD_LOCK,0);
	   scanf_slave_machine();
	for(i=1;i<=slave[0];i++){set_statuslist_1(slave[i],0,0,0);set_statuslist_2(slave[i],0,0,0);}//��ʼ������״̬���  
		  	   init=0;
		}
	   if(go==0)
	  { myled(); 
	   delay_time(1);
	   mybox.myid=AT24CXX_ReadOneByte(0x0010);
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
		if(scan_time>60*second)
			{
			led_on_off(ALL_NODE_LCD_LOCK,0); 
			scanf_slave_machine();
			scan_time=0;
                    	for(i=1;i<=slave[0];i++){set_statuslist_1(slave[i],0,0,0);set_statuslist_2(slave[i],0,0,0);}//��ʼ������״̬���  

		        }
	   }
			
			   
			     	

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
 if(led_lock==0)
 	{temperature();
 //key_lcd();
 key_idset();//��������ʾ����
  LIGHT(mystatus.work_status[0],mystatus.work_status[1]);//ˢָʾ�ƣ������ʾ������·�����˲� ����ɾ��
        }
//delay_ms(50);
}
}




/********************************************/
void SETID_task(void *pdata)
{
        OS_CPU_SR cpu_sr=0;  	    	
          while(1)
          	{
		  id_num=AT24CXX_ReadOneByte(0x0010);
	///	  id_num=1;//���Կ�����ʹ��
		if(id_num<1||id_num>33)
			{            		
                                      mybox.master=2;
			             OS_ENTER_CRITICAL();
                      		OSTaskSuspend(MASTER_TASK_PRIO );//����������״̬.
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
		 OSTaskResume(MASTER_TASK_PRIO );//������������״̬
		 OSTaskResume(MYLED_TASK_PRIO );//������ʾ����״̬
		 OSTaskResume(Receive_TASK_PRIO );//�����ӻ�����״̬
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
gonglvyinshu();//���㹦�ʣ���ѹ��������ʾ�����ֿ�
temperature();
key_lcd();
delay_time(1);
delay_ms(50);//û����ʱ����������
}
}


/******************************************/


 /***********************************/

int slave_control(u8 i,u8 j)//������λ����ָ��	 
{ 
 if(mybox.send==0)//��̬����
   	{
	   //LED1=!LED1;
	return 0;
    }
   if(mybox.send==1&&auto_on==1) //��λ������
   	{ 	 // LED1=!LED1;
 if(i==1&&j==1)
 {GPIO_ResetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	  led_lock=0; //������ɿ���
 }
 if(i==1&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	  led_lock=0; //������ɿ���
 }
 if(i==2&&j==1)
 	{GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	  led_lock=0; //������ɿ���

 }
 if(i==2&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	  led_lock=0; //������ɿ���

 }
	return 1;
   	}
if(mybox.send==2)//�鿴�ӻ�״̬
 {
  status_trans_rs485(&mystatus);
 led_lock=0;//������ɿ���
return 2;
 }
if(mybox.send==3&&auto_on==1)//�鿴�ӻ�״̬
{
  status_trans_rs485_dis(&mystatus);
 led_lock=0;//������ɿ���
return 2;
 }
if(mybox.send==4&&auto_on==1)//��ʼͶ���ʱʹ�ã���֤��Ͷ��ȥ������������
{
{ 	 // LED1=!LED1;
 if(i==1&&j==1)
 {GPIO_ResetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	  led_lock=0; //������ɿ���
 }
 if(i==1&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	  led_lock=0; //������ɿ���
 }
 if(i==2&&j==1)
 	{GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	  led_lock=0; //������ɿ���

 }
 if(i==2&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	  led_lock=0; //������ɿ���

 }
   	}
 status_trans_rs485_RT();//�ӻ�����
 led_lock=0;//������ɿ���
return 2;
 }

 if(mybox.send==ALL_NODE_LCD_UNLOCK) //��ˢ��led��Ļ
 {
 led_lock=0;
 return 3;
 }
 if(mybox.send==ALL_NODE_LCD_LOCK) //�ر�ˢ��led��Ļ
 {
  led_lock=1;
 return 4;
 }
  if(mybox.send==IDLE_NODE_LCD_LOCK) //����п��е������ر�ˢ��led��Ļ
 {
 if(mystatus.work_status[0]==0||mystatus.work_status[1]==0){led_lock=1;return 4;}
 }
 if(mybox.send==BUSY_NODE_LCD_LCOK) //�����æµ�������ر�ˢ��led��Ļ
 {
 if(mystatus.work_status[0]==1||mystatus.work_status[1]==1){led_lock=1;return 4;}
 }
 
if((mybox.send-mybox.myid)==NODE_LCD_LOCK_BASE)
{
led_lock=1;
return 4;
}
/*
if(mybox.send>=Sub_Order&&mybox.send<=(Sub_Order+64))
{
    if(i==1&&j==1)
{
//led_lock=0;//������ɿ���
OSSemPost(sub_machine1_close);
}
    if(i==1&&j==0)
{
//led_lock=0;//������ɿ���
OSSemPost(sub_machine1_open);
}
    if(i==2&&j==1)
{
//led_lock=0;//������ɿ���
OSSemPost(sub_machine2_close);
}
    if(i==2&&j==0)
{
//led_lock=0;//������ɿ���
OSSemPost(sub_machine2_open);
}												
return 5;
}
*/

return 6; //����ʧ��
}
/*
void sub_machine1_close_task(void *pdate)//�ӻ�����
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
    //      sub_delaytime_15(mybox.send-Sub_Order);
			//	time=0;
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	true_worktime1_flag=1;
             LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
                  led_lock=0;//������ɿ���

         }

}
void sub_machine1_open_task(void *pdate)//�ӻ�����
{   u8 err;
    // u16 time=0;
    while(1)
    	{
        OSSemPend(sub_machine1_open,0,&err);
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	true_worktime1_flag=0;
	LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	led_lock=0;//������ɿ���
          {  
		 delay_ms(6000);
		 delay_ms(6000);		 
	set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);

         }
          	 	
	}

}


void sub_machine2_close_task(void *pdate)//�ӻ�����
{   u8 err;
    while(1)
    	{
        OSSemPend(sub_machine2_close,0,&err);
set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);        
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	true_worktime2_flag=1;
        LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
		led_lock=0;//������ɿ���

	}

}



void sub_machine2_open_task(void *pdate)//�ӻ�����
{   u8 err;
    while(1)
    	{
        OSSemPend(sub_machine2_open,0,&err);
//          sub_delaytime_5(mybox.send-Sub_Order);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	true_worktime2_flag=0;
	LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
           led_lock=0;//������ɿ���
		{
		delay_ms(6000);
		delay_ms(6000);
	set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);				
	        }
	}

}
*/
 void scanf_task(void *pdate)
 	{ u8 err;
while(1)
{
        OSSemPend(scan_slave,0,&err);
       led_on_off(ALL_NODE_LCD_LOCK,0);
             scanf_slave_machine();
 

}
}


void EXTI_Configuration(void)//��ʼ������

{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//��ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE );	  //ʹ��ADC1ͨ��ʱ��

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	 		
	//ʹ���ⲿ�жϸ���ʱ��
	
	//ӳ��GPIOE��Pin0��EXTILine0
GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);



EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);         	//Ƕ�׷���Ϊ��0
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;      	//�ж�ͨ��Ϊͨ��10
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //�������ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;    		//��Ӧ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     		//���ж�
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

