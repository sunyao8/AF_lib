
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
#include "lcd.h"//²âÊÔÓÃ
/////////////////////////UCOSIIÈÎÎñÉèÖÃ///////////////////////////////////
//START ÈÎÎñ
//ÉèÖÃÈÎÎñÓÅÏÈ¼¶
#define START_TASK_PRIO      			10 //¿ªÊ¼ÈÎÎñµÄÓÅÏÈ¼¶ÉèÖÃÎª×îµÍ
//ÉèÖÃÈÎÎñ¶ÑÕ»´óĞ¡
#define START_STK_SIZE  				64
//ÈÎÎñ¶ÑÕ»	
OS_STK START_TASK_STK[START_STK_SIZE];
//ÈÎÎñº¯Êı
void start_task(void *pdata);	
 			   
//LEDÈÎÎñ
//ÉèÖÃÈÎÎñÓÅÏÈ¼¶
#define TAKE_TASK_PRIO       			7 
//ÉèÖÃÈÎÎñ¶ÑÕ»´óĞ¡
#define TAKE_STK_SIZE  		    		64
//ÈÎÎñ¶ÑÕ»
OS_STK TAKE_TASK_STK[TAKE_STK_SIZE];
//ÈÎÎñº¯Êı
void Heartbeat_task(void *pdata);

//½ÓÊÕÈÎÎñ
//ÉèÖÃÈÎÎñÓÅÏÈ¼¶
#define Receive_TASK_PRIO       			8 
//ÉèÖÃÈÎÎñ¶ÑÕ»´óĞ¡
#define Receive_STK_SIZE  		    		64
//ÈÎÎñ¶ÑÕ»
OS_STK Receive_TASK_STK[Receive_STK_SIZE];
//ÈÎÎñº¯Êı
void  Receive_task(void *pdata);





//Ö÷ÈÎÎñ
//ÉèÖÃÈÎÎñÓÅÏÈ¼¶
 #define MAIN_TASK_PRIO 4 
//ÉèÖÃÈÎÎñ¶ÑÕ»´óĞ¡
#define MAIN_STK_SIZE  					128
//ÈÎÎñ¶ÑÕ»	
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//ÈÎÎñº¯Êı
void main_task(void *pdata);

//ĞÅºÅÁ¿¼¯ÈÎÎñ
//ÉèÖÃÈÎÎñÓÅÏÈ¼¶

//ÉèÖÃÈÎÎñ¶ÑÕ»´óĞ¡
#define MASTER_STK_SIZE  		 		64
//ÈÎÎñ¶ÑÕ»	
OS_STK MASTER_TASK_STK[MASTER_STK_SIZE];
//ÈÎÎñº¯Êı
 void master_task(void *pdata);
 

#define MYLED_TASK_PRIO       			13 
//ÉèÖÃÈÎÎñ¶ÑÕ»´óĞ¡
#define MYLED_STK_SIZE  		    		64
//ÈÎÎñ¶ÑÕ»
OS_STK MYLED_TASK_STK[MYLED_STK_SIZE];
//ÈÎÎñº¯Êı
void myled_task(void *pdata);

#define SETID_TASK_PRIO       			2 
//ÉèÖÃÈÎÎñ¶ÑÕ»´óĞ¡
#define SETID_STK_SIZE  		    		64
//ÈÎÎñ¶ÑÕ»
OS_STK SETID_TASK_STK[SETID_STK_SIZE];
//ÈÎÎñº¯Êı
void SETID_task(void *pdata);



  	   
extern box mybox;
  
extern  u16  dog_clock;

extern OS_EVENT * RS485_MBOX,*RS485_STUTAS_MBOX;			//	rs485ÓÊÏäĞÅºÅÁ¿

extern OS_EVENT *Heartbeat;			 //ĞÄÌøĞÅºÅÁ¿
extern OS_EVENT *master_led_task;
extern u8 cont;//ÓÃÓÚ¸ü¸ÄÖ÷»úºÅµÄ¼Ç´ÎÊıÆ÷
extern  u8 token[33];//Ö÷»úºÅÁîÅÆ

extern status_box mystatus;

extern status_list_node system_status_list_1[33];

extern status_list_node system_status_list_2[33];

extern idle_list sort_idle_list_1[33];
extern idle_list sort_idle_list_2[33];
extern busy_list sort_busy_list_1[33];
extern busy_list sort_busy_list_2[33];


extern u8 hguestnum,gonglvshishu;
extern u32 idle_time,dianliuzhi;
extern u16 wugongkvar;
extern s8 L_C_flag;
extern u8 id_num;
//½ÓÊÕ»º´æÇø

//////////////////////////////////////////

u8 led_lock=0;
u8 init=1;
int subcontrol(u8,u8);

//#define ID  1

#define SIZE_1 50
#define SIZE_2 25
#define WORK_STATUS_1	 0//0ÎªÃ»ÓĞ¹¤×÷  1Îª¹¤×÷  2Îª»µµô£¬³õÊ¼»¯Îª0
#define WORK_STATUS_2    0 
#define WORK_TIME_1 0
#define WORK_TIME_2	0
/////////////////////////////////////////////
int main(void)
 {	 
  
	 
	
	delay_init();	    	 //ÑÓÊ±º¯Êı³õÊ¼»¯	  
	NVIC_Configuration(); 	 //ÉèÖÃNVICÖĞ¶Ï·Ö×é2:2Î»ÇÀÕ¼ÓÅÏÈ¼¶£¬2Î»ÏìÓ¦ÓÅÏÈ¼¶
// 	LED_Init();			     //LED¶Ë¿Ú³õÊ¼»¯
/*************************/
		delay_us(500000);
	HT1621_Init();
	KEY_Init();          //³õÊ¼»¯Óë°´¼üÁ¬½ÓµÄÓ²¼ş½Ó¿Ú  
	AT24CXX_Init();			//IIC³õÊ¼»¯
	Adc_Init();
/************************************/

///		uart_init(9600);LCD_Init();	                                                              //µ÷ÊÔÏÔÊ¾
	RS485_Init(9600);	//³õÊ¼»¯RS485
	TIM4_Int_Init(9999,7199);//10KhzµÄ¼ÆÊıÆµÂÊ£¬¼ÆÊı5K´ÎÎª500ms 
	TIM3_Cap_Init(0XFFFF,72-1);	//ÒÔ1MhzµÄÆµÂÊ¼ÆÊı	  //¿ª·¢°åÓÃLCDÊ±£¬±ØĞë×¢µô´Ë¾Ù£¬Òı½Å³åÍ»
	 initmybox();
	 init_mystatus(SIZE_1,SIZE_2,WORK_STATUS_1,WORK_STATUS_2,WORK_TIME_1,WORK_TIME_2);
	 
	OSInit();  	 			//³õÊ¼»¯UCOSII
			  
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//´´½¨ÆğÊ¼ÈÎÎñ
	OSStart();	    
}							    
//¿ªÊ¼ÈÎÎñ
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;  	    
	pdata = pdata; 	
	 Heartbeat=OSSemCreate(0);
	 RS485_MBOX=OSMboxCreate((void*)0);
	 RS485_STUTAS_MBOX=OSMboxCreate((void*)0);
	OSStatInit();					//³õÊ¼»¯Í³¼ÆÈÎÎñ.ÕâÀï»áÑÓÊ±1ÃëÖÓ×óÓÒ	
 	OS_ENTER_CRITICAL();			//½øÈëÁÙ½çÇø(ÎŞ·¨±»ÖĞ¶Ï´ò¶Ï)    			   
 	OSTaskCreate(master_task,(void *)0,(OS_STK*)&MASTER_TASK_STK[MASTER_STK_SIZE-1],MASTER_TASK_PRIO);	 				   
 	OSTaskCreate(Heartbeat_task,(void *)0,(OS_STK*)&TAKE_TASK_STK[TAKE_STK_SIZE-1],TAKE_TASK_PRIO);
	OSTaskCreate(Receive_task,(void *)0,(OS_STK*)&Receive_TASK_STK[Receive_STK_SIZE-1],Receive_TASK_PRIO);
	OSTaskCreate(myled_task,(void *)0,(OS_STK*)&MYLED_TASK_STK[MYLED_STK_SIZE-1],MYLED_TASK_PRIO);
	OSTaskCreate(SETID_task,(void *)0,(OS_STK*)&SETID_TASK_STK[SETID_STK_SIZE-1],SETID_TASK_PRIO);
 	OSTaskSuspend(START_TASK_PRIO);	//¹ÒÆğÆğÊ¼ÈÎÎñ.
	OS_EXIT_CRITICAL();				//ÍË³öÁÙ½çÇø(¿ÉÒÔ±»ÖĞ¶Ï´ò¶Ï)
}
//LEDÈÎÎñ

 /**************´Ó»úÈÎÎñ**********************/
void Receive_task(void *pdate)//´Ó»úÈÎÎñ
{   u8 err;
	 u8 *msg;
	 int flag1,flag2;
	
    while(1)
    	{
		 if(mybox.master==1)OSTaskSuspend(Receive_TASK_PRIO);
	 msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//½ÓÊÕµ½ÓĞÊı¾İ
	 flag1=rs485_trans_order(msg);
	 dog_clock=10;	 
	 if(flag1==0);/***Ö÷»ú²»ÊÇ¸ø±¾»úÍ¨ĞÅ£¬ĞÅÏ¢ÉáÆú***/
	 if(flag1==1)/***ÊÇ±¾»úĞÅÏ¢***/
	 	{		//LED1=!LED1;	  
		       dog_clock=10;	 
	 	      flag2=subcontrol(mybox.relay,mybox.message);
		       if(flag2==0);/******ĞÄÌøĞÅÏ¢£¬Ö÷»ú»î×Å£¬²»±ØÆô¶¯´ÓĞÂ¶¨Î»Ö÷»úº¯Êı£¬·µ»Ø¸øÖ÷»ú±¾´Ó»úÒ²´æÔÚ    Î¹¹·*********/
               if(flag2==1);/*****ÏÂÎ»»ú£¬¿ØÖÆÃüÁî£¬Î¹¹·*****/		  
	 	}

      if(led_lock==0){temperature();key_lcd();}
	}
	}
 /**************Ö÷»úÈÎÎñ**********************/
  void master_task(void *pdata)	  //Ö÷»úÈÎÎñ
  {	  OS_CPU_SR cpu_sr=0;
      u8 go=0;
	  u32 i;
	  // u8 *msg,err;
	  u8 try_cont=0;
	  for(i=1;i<33;i++){set_statuslist_1(i,0,0,0);set_statuslist_2(i,0,0,0);}//³õÊ¼»¯Á½¸ö×´Ì¬¶ÓÁ
   while(1)
   	{
  	if(mybox.master==1)
     {	hguestnum=111;
	if(init==1)
		{
       delay_time(5000);//³õÊ¼»¯ÑÓÊ±£¬Ê¹ÏµÍ³ÎÈ¶¨
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
		 if(((gonglvshishu)>95)&&(L_C_flag==1))
		 	{
                        unload_power(system_status_list_1,system_status_list_2);
		       }
	        if((idle_time>5)&&(gonglvshishu>90&&gonglvshishu<95)&&(L_C_flag==1))
                {idle_time=0;turn_power(system_status_list_1,system_status_list_2);}
		if(L_C_flag==0){C_unload_power(system_status_list_1, system_status_list_2);}
	   }
		if(go==1)
		{
		{OSSemPost(Heartbeat);delay_ms(100);try_cont++;}  //°ÑÑÓÊ±·â×°³Éº¯Êı
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
				                   
                        	led_on_off(0);		 //¹Ø±Õ´Ó»úled¹¦ÄÜ£¬·ÀÖ¹Í¨ĞÅ¿ØÖÆĞÅÏ¢µÄ¶ªÊ§
			   for(i=1;i<33;i++)		 //ÊÕ¼¯´Ó»ú×´Ì¬
               { inquiry_slave_status(i);	
	         }
			   //ÏÂÃæÊÇÓÃLCDÆÁ²âÊÔ»ØÀ¡ĞÅÏ¢¹¦ÄÜ
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
				   if(system_status_list_1[i].work_status==0){order_trans_rs485(mybox.myid,i,1,1,1);  delay_us(10000);}//ÑéÖ¤´Ó»ú9µÄ¿ØÖÆºÍ¹¤×÷¼ÆÊ±
				   if(system_status_list_2[i].work_status==0){order_trans_rs485(mybox.myid,i,1,2,1);  delay_us(10000);}//				     
				 	}
                       
				
				     //´ò¿ª´Ó»úled¹¦ÄÜ
                        	led_on_off(1);
		       }
			 	if(go==3){// LED0=!LED0;
				  order_trans_rs485(mybox.myid,2,2,0,0);
				  delay_us(10000);
				  }
				
				if(go==4){ 
					s8 c=0,d=0;
                       myled() ;             
			led_on_off(0);		 //¹Ø±Õ´Ó»úled¹¦ÄÜ£¬·ÀÖ¹Í¨ĞÅ¿ØÖÆĞÅÏ¢µÄ¶ªÊ§

							 for(i=1;i<33;i++)
					{
			   if(system_status_list_1[i].work_status==0){order_trans_rs485(mybox.myid,i,1,1,1);  delay_us(10000);}//ÑéÖ¤´Ó»ú9µÄ¿ØÖÆºÍ¹¤×÷¼ÆÊ±
				   if(system_status_list_2[i].work_status==0){order_trans_rs485(mybox.myid,i,1,2,1);  delay_us(10000);}//				     
				 	}

			   for(i=1;i<33;i++)		 //ÊÕ¼¯´Ó»ú×´Ì¬
               { inquiry_slave_status(i);	
	         }
                             c=sort_busynode_list_asc(sort_busy_list_2,system_status_list_2);
                             d=sort_busynode_list_asc(sort_busy_list_1, system_status_list_1);
			   //ÏÂÃæÊÇÓÃLCDÆÁ²âÊÔ»ØÀ¡ĞÅÏ¢¹¦ÄÜ
				 for(i=1;i<=d;i++)
				   {
				   LCD_ShowxNum(5+1*25,i*15,sort_busy_list_1[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+2*25,i*15,sort_busy_list_1[i].size,3,16,0X80);
				   LCD_ShowxNum(5+3*25,i*15,sort_busy_list_1[i].work_time,3,16,0X80);	
			 //         LCD_ShowxNum(5+4*25,i*15,idle_time,3,16,0X80);
//				    LCD_ShowxNum(5+5*25,i*15,dianliuzhi,3,16,0X80);
  
					}
					for(i=1;i<=c;i++)
					{
				   LCD_ShowxNum(5+6*25,i*15,sort_busy_list_2[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+7*25,i*15,sort_busy_list_2[i].size,3,16,0X80);
				   LCD_ShowxNum(5+8*25,i*15,sort_busy_list_2[i].work_time,3,16,0X80);	
			       
				   }

		//		 d=sort_idlenode_list(sort_idle_list_2,system_status_list_2);			 

		//		    for(i=1;i<=d;i++)
			//	   {
                         //         LCD_ShowxNum(5+1*25,i*15,sort_idle_list_2[i].myid,3,16,0X80);
				//   LCD_ShowxNum(5+2*25,i*15,sort_idle_list_2[i].size,3,16,0X80);

//				   }
				 	
/*							 for(i=1;i<33;i++)
					{
			   if(system_status_list_1[i].work_status==1){order_trans_rs485(mybox.myid,i,1,1,0);  delay_us(10000);}//ÑéÖ¤´Ó»ú9µÄ¿ØÖÆºÍ¹¤×÷¼ÆÊ±
				   if(system_status_list_2[i].work_status==1){order_trans_rs485(mybox.myid,i,1,2,0);  delay_us(10000);}//				     
				 	}

    */                   
				
				     //´ò¿ª´Ó»úled¹¦ÄÜ
                        	led_on_off(1);
		 

					}

		if(go==5)
			{

                                                      s8 d=0,c=0;
								  led_on_off(0);
						   for(i=1;i<33;i++)		 //ÊÕ¼¯´Ó»ú×´Ì¬
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
				 led_on_off(1);	
		}

     if(go==6)
     	{
          	while(1)
	{
               delay_time(1);
		   myled(); 
     // order_trans_rs485(mybox.myid,mybox.myid,1,1,1);
	//	 delay_us(1000000);
		// order_trans_rs485(mybox.myid,mybox.myid,1,1,0);
		// delay_us(1000000);
		// order_trans_rs485(mybox.myid,mybox.myid,1,2,1);
		// delay_us(1000000);
		// order_trans_rs485(mybox.myid,mybox.myid,1,2,0);
		// delay_us(1000000);
	        // order_trans_rs485(mybox.myid,mybox.myid,1,1,1);
       // order_trans_rs485(mybox.myid,mybox.myid,1,2,1);
	 


	  
	 
	}	 

	 }


	}
		                                      //Æô¶¯½ÓÊÕ³Ì
	if(mybox.master==0)
	{
		OS_ENTER_CRITICAL();
		 OSTaskResume(Receive_TASK_PRIO );//Æô¶¯´Ó»úÈÎÎñ×´Ì¬
		OSTaskSuspend(MASTER_TASK_PRIO );//¹ÒÆğÖ÷»úÈÎ×´Ì¬.
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
	{temperature();key_lcd();}
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
		if(id_num<1||id_num>33)
			{            		
                                      mybox.master=2;
			             OS_ENTER_CRITICAL();
                      		OSTaskSuspend(MASTER_TASK_PRIO );//¹ÒÆğÖ÷»úÈÎ×´Ì¬.
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
		 OSTaskResume(MASTER_TASK_PRIO );//Æô¶¯Ö÷»úÈÎÎñ×´Ì¬
		 OSTaskResume(MYLED_TASK_PRIO );//Æô¶¯ÏÔÊ¾ÈÎÎñ×´Ì¬
		 OSTaskResume(Receive_TASK_PRIO );//Æô¶¯´Ó»úÈÎÎñ×´Ì¬
               OSTaskSuspend(SETID_TASK_PRIO);
			   OS_EXIT_CRITICAL();	
			 }

		  }

}

/******************************************/


 /***********************************/

int subcontrol(u8 i,u8 j)//¸øÏÂÏÂÎ»»ú·ÅÖ¸Áî	 
{ 
 if(mybox.send==0)//ĞÄÌ¬Âö²«
   	{
	   //LED1=!LED1;
	return 0;
    }
   if(mybox.send==1) //ÏÂÎ»»ú¿ØÖÆ
   	{ 	 // LED1=!LED1;
   	if(i==1&&j==1){GPIO_ResetBits(GPIOA,GPIO_Pin_0);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);LED0=!LED0; }
    if(i==1&&j==0){GPIO_SetBits(GPIOA,GPIO_Pin_0);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);}
    if(i==2&&j==1){GPIO_ResetBits(GPIOA,GPIO_Pin_8);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);LED0=!LED0; }
    if(i==2&&j==0){GPIO_SetBits(GPIOA,GPIO_Pin_8);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);}
      LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	return 1;
   	}
if(mybox.send==2)//²é¿´´Ó»ú×´Ì¬
 {
 //	LED1=!LED1;
  status_trans_rs485(&mystatus);
return 2;
 }
 if(mybox.send==3) //¹Ø±ÕË¢ĞÂledÆÁÄ»
 {
 led_lock=1;
 return 3;
 }
 if(mybox.send==4) //´ò¿ªË¢ĞÂledÆÁÄ»
 {
  led_lock=0;
 return 4;
 }
return 5; //²Ù×÷Ê§°Ü
}


 
