#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "485ANN.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "key.h"
/***********************************************************************/
u16 RS485_RX_BUF[64]; 		//½ÓÊÕ»º³å,×î´ó64¸ö×Ö½Ú
//½ÓÊÕµ½µÄÊı¾İ³¤¶È
u8 RS485_RX_CNT=0;  
//Ä£Ê½¿ØÖÆ
 u16  dog_clock=10;

 OS_EVENT * RS485_MBOX,* RS485_STUTAS_MBOX;			//	rs485ÓÊÏäĞÅºÅÁ¿
 OS_EVENT *Heartbeat;			 //ĞÄÌøĞÅºÅÁ¿
OS_EVENT *master_led_task;
u8 cont=0;//ÓÃÓÚ¸ü¸ÄÖ÷»úºÅµÄ¼Ç´ÎÊıÆ÷
u32 life_time_1=0;
u32 life_time_2=0;
u8 turn_flag=1;//ÂÖĞİÊ¹ÓÃ±äÁ¿
s8 turn_label_idle1=0,turn_label_idle2=0;//ÂÖĞİÊ¹ÓÃ±äÁ¿

box mybox;
status_box mystatus;
idle_list sort_idle_list_1[33];
idle_list sort_idle_list_2[33];
busy_list sort_busy_list_1[33];
busy_list sort_busy_list_2[33];

status_list_node system_status_list_1[33];
status_list_node system_status_list_2[33];

//u8 rs485buf[LEN_control];//·¢ËÍ¿ØÖÆĞÅÏ¢
u8 rs485buf[LEN_control];//·¢ËÍ¿ØÖÆĞÅÏ¢
u8 statusbuf[LEN_status];//·¢ËÍ×´Ì¬ĞÅÏ¢

u32 idle_time=0;
/****************************************************************/
u8 si[]={0,2,3,5,7,9,10,12,14,16,        //0~9
	   	17,19,21,22,24,26,28,29,31,33,  //10~19
		34,36,37,39,41,42,44,45,47,48,  //20~29
		50,52,53,54,56,57,59,60,62,63,	//30~39
		64,66,67,68,69,71,72,73,74,75,	//40~49
		77,78,79,80,81,82,83,84,85,86,	//50~59
		87,87,88,89,90,91,91,92,93,93,	//60~69
		94,95,95,96,96,97,97,97,98,98,	//70~79
		98,99,99,99,99,100,100,100,100,100,100	//80~90
		};
u8 co[]={100,100,100,100,100,99,99,99,99,98, //0~9
          98,98,97,97,97,96,96,95,95,94,	  //10~19
          93,93,92,91,91,90,89,88,87,87,	  //20~29
          86,85,84,83,82,81,80,79,78,77,	  //30~39
          75,74,73,72,71,69,68,67,66,64,	  //40~49
          63,62,60,59,57,56,54,53,52,50,	  //50~59
          48,47,45,44,42,41,39,37,36,34,	  //60~69
          33,31,29,28,26,24,22,21,19,17,	  //70~79
          16,14,12,10,9,7,5,3,2,0			  //80~90
		  };


/**********************²âÊÔÎŞ¹¦¹¦ÂÊ Êı¾İ*************************************/

s16 wogong_try=50;

/************************************************************/


extern u8 id_num; 
extern u8 grafnum,tempshuzhi,gonglvshishu;
extern u16 dianya_zhi,	wugongkvar,k;
extern u32	dianliuzhi;
/*****************************************************/
 void TIM4_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //Ê±ÖÓÊ¹ÄÜ
	
	//¶¨Ê±Æ÷TIM4³õÊ¼»¯
	TIM_TimeBaseStructure.TIM_Period = arr; //ÉèÖÃÔÚÏÂÒ»¸ö¸üĞÂÊÂ¼ş×°Èë»î¶¯µÄ×Ô¶¯ÖØ×°ÔØ¼Ä´æÆ÷ÖÜÆÚµÄÖµ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //ÉèÖÃÓÃÀ´×÷ÎªTIMxÊ±ÖÓÆµÂÊ³ıÊıµÄÔ¤·ÖÆµÖµ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ÉèÖÃÊ±ÖÓ·Ö¸î:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIMÏòÉÏ¼ÆÊıÄ£Ê½
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //¸ù¾İÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯TIMxµÄÊ±¼ä»ùÊıµ¥Î»
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //Ê¹ÄÜÖ¸¶¨µÄTIM4ÖĞ¶Ï,ÔÊĞí¸üĞÂÖĞ¶Ï

	//ÖĞ¶ÏÓÅÏÈ¼¶NVICÉèÖÃ
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4ÖĞ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //ÏÈÕ¼ÓÅÏÈ¼¶0¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //´ÓÓÅÏÈ¼¶3¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQÍ¨µÀ±»Ê¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);  //³õÊ¼»¯NVIC¼Ä´æÆ÷


	TIM_Cmd(TIM4, ENABLE);  //Ê¹ÄÜTIMx					 
}
 
 void TIM4_IRQHandler(void)   //TIM4ÖĞ¶Ï
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //¼ì²éTIM4¸üĞÂÖĞ¶Ï·¢ÉúÓë·ñ
		{	  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //Çå³ıTIMx¸üĞÂÖĞ¶Ï±êÖ¾
	if(mybox.master==0)	
		{
		
		if(dog_clock==0)
		   { 
		   // LED0=!LED0;
			turn_master_id(mybox.myid);
			  cont++;
			}
			if(dog_clock>0){dog_clock--;cont=1;}
		 }
		 if (mystatus.work_status[0]==1)  //¹¤×÷Ê±¼äµÄ¼ÆÊ±
		    {  life_time_1++;
		       if(life_time_1==10)
			   { mystatus.work_time[0]++;
			     life_time_1=0;
				  }
			        if(mystatus.work_time[0]==37)mystatus.work_time[0]=39;
				  if(mystatus.work_time[0]==254)mystatus.work_time[0]=0;
			   }
		 	
		 if(mystatus.work_status[1]==1)
		 	{ life_time_2++;
              if(life_time_2==10)
			  	{  mystatus.work_time[1]++;
			       life_time_2=0;
              	         }   
                   if(mystatus.work_time[1]==37)mystatus.work_time[1]=39;
			 if(mystatus.work_time[1]==254)mystatus.work_time[1]=0;	   
			    
		     }
		   if (mystatus.work_status[0]==0)  //¹¤×÷Ê±¼äÇåÁã
		    {   mystatus.work_time[0]=0;}
		    if (mystatus.work_status[1]==0)  //¹¤×÷Ê±¼äÇåÁã
		    {   mystatus.work_time[1]=0;}

			if(mybox.master==1)
		       {  idle_time++;
			if(idle_time==65535)idle_time=0;
			}
		}
   	OSIntExit();  
}
//////////////////////////////////////////////////////////////////////////////////
void TIM3_Cap_Init(u16 arr,u16 psc)
{	NVIC_InitTypeDef NVIC_InitStructure;	 
	RCC->APB1ENR|=1<<1;   	//TIM3 Ê±ÖÓÊ¹ÄÜ 
	RCC->APB2ENR|=1<<3;    	//Ê¹ÄÜPORTBÊ±ÖÓ  
	 
	GPIOB->CRL&=0XFFFFFF00;	//PB0 PB1Çå³ıÖ®Ç°ÉèÖÃ  
	GPIOB->CRL|=0X00000088;	//PB0 PB1ÊäÈë   
	GPIOB->ODR&=~(1<<0);		//PB0 PB1ÏÂÀ­
	GPIOB->ODR&=~(1<<1);
	  
 	TIM3->ARR=arr;  		//Éè¶¨¼ÆÊıÆ÷×Ô¶¯ÖØ×°Öµ   
	TIM3->PSC=psc;  		//Ô¤·ÖÆµÆ÷ 

	TIM3->CCMR2|=1<<0;		//CC3S=01 	Ñ¡ÔñÊäÈë¶Ë IC3Ó³Éäµ½TI3ÉÏ
 	TIM3->CCMR2|=0<<4; 		//IC3F=0000 ÅäÖÃÊäÈëÂË²¨Æ÷ ²»ÂË²¨
 	TIM3->CCMR2|=0<<2;  	//IC3PS=00 	ÅäÖÃÊäÈë·ÖÆµ,²»·ÖÆµ 

	TIM3->CCER|=0<<9; 		//CC3P=0	ÉÏÉıÑØ²¶»ñ1
	TIM3->CCER|=1<<8; 		//CC3E=1 	ÔÊĞí²¶»ñ¼ÆÊıÆ÷1µÄÖµµ½²¶»ñ¼Ä´æÆ÷ÖĞ

	TIM3->DIER|=1<<3;   	//ÔÊĞí²¶»ñ3ÖĞ¶Ï				
	TIM3->DIER|=1<<0;   	//ÔÊĞí¸üĞÂ1ÖĞ¶Ï	

	TIM3->CCMR2|=1<<8;		//CC4S=01 	Ñ¡ÔñÊäÈë¶Ë IC4Ó³Éäµ½TI4ÉÏ
 	TIM3->CCMR2|=0<<12; 	//IC4F=0000 ÅäÖÃÊäÈëÂË²¨Æ÷ ²»ÂË²¨
 	TIM3->CCMR2|=0<<10; 	//IC4PS=00 	ÅäÖÃÊäÈë·ÖÆµ,²»·ÖÆµ 

	TIM3->CCER|=0<<13; 		//CC4P=0	ÉÏÉıÑØ²¶»ñ
	TIM3->CCER|=1<<12; 		//CC4E=1 	ÔÊĞí²¶»ñ¼ÆÊıÆ÷µÄÖµµ½²¶»ñ¼Ä´æÆ÷ÖĞ

	TIM3->DIER|=1<<4;   	//ÔÊĞí²¶»ñ4ÖĞ¶Ï				
	TIM3->DIER|=1<<0;   	//ÔÊĞí¸üĞÂÖĞ¶Ï
	TIM3->CR1|=0x01;    	//Ê¹ÄÜ¶¨Ê±Æ÷2
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3ÖĞ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //ÏÈÕ¼ÓÅÏÈ¼¶0¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  //´ÓÓÅÏÈ¼¶4¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQÍ¨µÀ±»Ê¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);  //³õÊ¼»¯NVIC¼Ä´æÆ÷
	   
}

//²¶»ñ×´Ì¬
//[7]:0,Ã»ÓĞ³É¹¦µÄ²¶»ñ;1,³É¹¦²¶»ñµ½Ò»´Î.
//[6]:0,»¹Ã»²¶»ñµ½¸ßµçÆ½;1,ÒÑ¾­²¶»ñµ½¸ßµçÆ½ÁË.
//[5:0]:²¶»ñ¸ßµçÆ½ºóÒç³öµÄ´ÎÊı
u8  TIM3CH1_CAPTURE_STA=0;	//ÊäÈë²¶»ñ×´Ì¬		    				
u16	TIM3CH1_CAPTURE_VAL;	//ÊäÈë²¶»ñÖµ
u16	TIM3CH1_CAPTURE_PHA;	//ÊäÈë²¶»ñÖµ
//¶¨Ê±Æ÷3ÖĞ¶Ï·şÎñ³ÌĞò	 
void TIM3_IRQHandler(void)
{ 		    
	u16 tsr;
	tsr=TIM3->SR;
 	if((TIM3CH1_CAPTURE_STA&0X80)==0)//»¹Î´³É¹¦²¶»ñ	
	{
		if(tsr&0X01)//Òç³ö
		{	    
			if(TIM3CH1_CAPTURE_STA&0X40)//ÒÑ¾­²¶»ñµ½¸ßµçÆ½ÁË
			{
				if((TIM3CH1_CAPTURE_STA&0X3F)==0X3F)//¸ßµçÆ½Ì«³¤ÁË
				{
					TIM3CH1_CAPTURE_STA|=0X80;//±ê¼Ç³É¹¦²¶»ñÁËÒ»´Î
					TIM3CH1_CAPTURE_VAL=0XFFFF;
				}else TIM3CH1_CAPTURE_STA++;
			}	 
		}
		if(tsr&0x10)//²¶»ñ1·¢Éú²¶»ñÊÂ¼ş
		{	
			if(TIM3CH1_CAPTURE_STA&0X40)		//²¶»ñµ½Ò»¸öÏÂ½µÑØ 		
			{	  			
				TIM3CH1_CAPTURE_STA|=0X80;		//±ê¼Ç³É¹¦²¶»ñµ½Ò»´Î¸ßµçÆ½Âö¿í
			    TIM3CH1_CAPTURE_VAL=TIM3->CCR4;	//»ñÈ¡µ±Ç°µÄ²¶»ñÖµ.
	 			TIM3->CCER&=~(1<<1);			//CC1P=0 ÉèÖÃÎªÉÏÉıÑØ²¶»ñ
			}else  								//»¹Î´¿ªÊ¼,µÚÒ»´Î²¶»ñÉÏÉıÑØ
			{
				TIM3CH1_CAPTURE_STA=0;			//Çå¿Õ
				TIM3CH1_CAPTURE_VAL=0;
				TIM3CH1_CAPTURE_STA|=0X40;		//±ê¼Ç²¶»ñµ½ÁËÉÏÉıÑØ
	 			TIM3->CNT=0;					//¼ÆÊıÆ÷Çå¿Õ
			  	TIM3->CCER&=~(1<<1);			//CC1P=0 ÉèÖÃÎªÉÏÉıÑØ²¶»ñ
			}		    
		}
		if(tsr&0x08)
		{
		 	if(TIM3CH1_CAPTURE_STA&0X40)		//²¶»ñµ½Ò»¸öÏÂ½µÑØ 		
			{	  			
			    TIM3CH1_CAPTURE_PHA=TIM3->CCR3;	//»ñÈ¡µ±Ç°µÄ²¶»ñÖµ.
			}
		}			     	    					   
 	}
	TIM3->SR=0;//Çå³ıÖĞ¶Ï±êÖ¾Î» 	    
}


//////////////////////////////////////////////////////////
		void USART2_IRQHandler(void)
{
 
	u8 RS485_RX_BUF[64];
	#ifdef OS_TICKS_PER_SEC	 	//Èç¹ûÊ±ÖÓ½ÚÅÄÊı¶¨ÒåÁË,ËµÃ÷ÒªÊ¹ÓÃucosIIÁË.
	OSIntEnter();    
      #endif
 		
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //½ÓÊÕµ½Êı¾İ
	{	 
	 			 
		 RS485_RX_BUF[RS485_RX_CNT++]=USART_ReceiveData(USART2); 	//¶ÁÈ¡½ÓÊÕµ½µÄÊı¾İ
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='&'){RS485_RX_BUF[0]='&'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='*')
		{
				RS485_RX_CNT=0;

				
				if(RS485_RX_BUF[1]=='#'){OSMboxPost(RS485_STUTAS_MBOX,(void*)&RS485_RX_BUF);}
				  else OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);
		} 
	}  	
	#ifdef OS_TICKS_PER_SEC	 	//Èç¹ûÊ±ÖÓ½ÚÅÄÊı¶¨ÒåÁË,ËµÃ÷ÒªÊ¹ÓÃucosIIÁË.
	OSIntExit();  											 
#endif

} 

void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			//ÉèÖÃÎª·¢ËÍÄ£Ê½
  	for(t=0;t<len;t++)		//Ñ­»··¢ËÍÊı¾İ
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);		
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;				//ÉèÖÃÎª½ÓÊÕÄ£Ê½	

}

void initmybox(u8 id)//³õÊ¼»¯×ÔÉíĞÅÏ¢
{  	 
  
 // for(i=1;i<33;i++)token[i]=0;//³õÊ¼»¯ÁîÅÆ
  mybox.master=0;
 //  token[1]=1;
 mybox.start='&';
 mybox.myid=id;
 mybox.source=0;
 mybox.destination=0;
 mybox.send=0;
 mybox.relay=0;
 mybox.message=0;
 mybox.end='*';	
						
}

void turn_master_id(u8 id)//¸Ä±äµ±Ç°Õû¸öÏµÍ³ÖĞÖ÷»úµÄIDºÅ
{
   u8 i,flag=0;
	{ 
	  flag=cont;
      if(id==(flag)){
	  	for(i=1;i<33;i++)
			{ order_trans_rs485(mybox.myid,i,0,0,0);
		     delay_us(10000);
			}//¼°Ê±¸æÖªÆäËûslave»úÆ÷£¬ÒÑÓĞÖ÷»ú
         mybox.master=1;
	    OSTaskResume(MASTER_TASK_PRIO);
		cont=1;
	  }
	 //  LED1=!LED1;
	  
      }
   }






 int rs485_trans_order(u8 *tx_r485)//½âÎöÓÉÖ÷»ú·¢ËÍ¹ıÀ´µÄĞÅºÅ£¬²¢·¢ËÍ¸øÏÂÎ»»ú
{ 
  dianya_zhi=comp_16(tx_r485[6],tx_r485[7]);
  dianliuzhi=comp_16(tx_r485[8],tx_r485[9]);
  wugongkvar=comp_16(tx_r485[10],tx_r485[11]);
  tempshuzhi=tx_r485[12];
  gonglvshishu=tx_r485[13];
   if(mybox.myid==tx_r485[2]||tx_r485[2]==0)//ÅĞ¶ÏÊÇ·ñÊÇ·¢¸ø±¾»úµÄĞÅÏ¢»òÊÇ¹ã²¥ĞÅÏ¢
   	{
   	 mybox.source=tx_r485[1];
   	 mybox.send=tx_r485[3];
     mybox.relay=tx_r485[4];
     mybox.message=tx_r485[5];
     return 1;
   	}
   else return 0;
}

 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message)//Ö÷»ú³ÌĞò£¬Ö÷»úÃüÁî½âÎö³ÉRS485ĞÅÏ¢£¬·¢ËÍ¸øÄ¿µÄ´Ó»ú
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
      rs485buf[0]='&';//Ğ­ÒéÍ·
	rs485buf[1]=source;
	rs485buf[2]=destination;
	rs485buf[3]=send;
	rs485buf[4]=relay;
	rs485buf[5]=message;
	rs485buf[6]=(dianya_zhi & (uint16_t)0x00FF);
	rs485buf[7]=((dianya_zhi & (uint16_t)0xFF00)>>8);
	rs485buf[8]=(dianliuzhi& (uint16_t)0x00FF);
	rs485buf[9]=((dianliuzhi& (uint16_t)0xFF00)>>8);
	rs485buf[10]=(wugongkvar& (uint16_t)0x00FF);
	rs485buf[11]=((wugongkvar& (uint16_t)0xFF00)>>8);
	rs485buf[12]=tempshuzhi;
	rs485buf[13]=gonglvshishu;
	rs485buf[14]='*';//Ğ­ÒéÎ²
	RS485_Send_Data(rs485buf,15);//·¢ËÍ5¸ö×Ö½Ú
	if(destination==source){mybox.send=send;subcontrol(relay, message);}//Èç¹ûĞÅÏ¢·¢¸øµÄ×Ô¼º
	OS_EXIT_CRITICAL();	
}

u16 comp_16(u16 a,u16 b)
{
u16 value=0;
value=((a&0x00FF)+((b<<8)&0xFF00));
return value;
}


void Heartbeat_task(void *pdata)//masterÈÎÎñ·¢ËÍÈÎÎñ
{		// u8 key;
        u8 i=0;
		u8 err;
	//  u8 rs485buf[5];
	while(1)
	{
	OSSemPend(Heartbeat,0,&err);
		for(i=1;i<10;i++)
		{	
	       order_trans_rs485(mybox.myid,0,0,0,0);
		    delay_us(10000);
		  // LCD_ShowxNum(60+i*32,190,0,7,16,0X80);
		}		 
				   
	}
}

void heartbeat(u8 t)
{	u8 i;
for(i=0;i<=t;i++)
		{	
	       order_trans_rs485(mybox.myid,0,0,0,0);
		    delay_us(10000);
		  // LCD_ShowxNum(60+i*32,190,0,7,16,0X80);
		}	
}










void led_on_off(u8 on_off) //²ÎÊıÖµ1 Îª´ò¿ªled £¬0Îª¹Ø±Õled
{
u8 i;
if(on_off==1)
    {
        for(i=1;i<80;i++)
	   { order_trans_rs485(mybox.myid,0,4,0,0);
             delay_us(10000);
          }	 
    }
if(on_off==0)
    {
        for(i=1;i<80;i++)
	   { order_trans_rs485(mybox.myid,0,3,0,0);
             delay_us(10000);
          }	 
    }
}


/*****************************»ØÀ¡ĞÅÏ¢º¯Êı********************************************/


void init_mystatus(u8 myid,u8 size_1,u8 size_2,u8 work_status_1,u8 work_status_2,u8 work_time_1,u8 work_time_2)
{
mystatus.myid=myid;
mystatus.size[0]=size_1;
mystatus.size[1]=size_2;
mystatus.work_status[0]=work_status_1;
mystatus.work_status[1]=work_status_2;
mystatus.work_time[0]=work_time_1;
mystatus.work_time[1]=work_time_2;
}



void set_now_mystatus(u8 myid,u8 size_1,u8 size_2,u8 work_status_1,u8 work_status_2,u8 work_time_1,u8 work_time_2)
{
mystatus.myid=myid;
mystatus.size[0]=size_1;
mystatus.size[1]=size_2;
mystatus.work_status[0]=work_status_1;
mystatus.work_status[1]=work_status_2;
mystatus.work_time[0]=work_time_1;
mystatus.work_time[1]=work_time_2;

}



 void status_trans_rs485(status_box *mystatus)//´Ó»ú³ÌĞò
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    statusbuf[0]='&';
	statusbuf[1]='#';
	statusbuf[2]=mystatus->myid;
	statusbuf[3]=mystatus->size[0];
	statusbuf[4]=mystatus->size[1];
	statusbuf[5]=mystatus->work_status[0];
	statusbuf[6]=mystatus->work_status[1];
	statusbuf[7]=mystatus->work_time[0];
	statusbuf[8]=mystatus->work_time[1];
	statusbuf[9]='*';
	RS485_Send_Data(statusbuf,10);//·¢ËÍ10¸ö×Ö½Ú
	OS_EXIT_CRITICAL();	
}


 void rs485_trans_status(u8 *tx_r485)//Ö÷»ú³ÌĞò£¬Ö÷»úÃüÁî½âÎö³ÉRS485ĞÅÏ¢£¬·¢ËÍ¸øÄ¿µÄ´Ó»ú
 	{
       system_status_list_1[tx_r485[2]].myid=tx_r485[2];
   	   system_status_list_1[tx_r485[2]].size=tx_r485[3];
   	   system_status_list_1[tx_r485[2]].work_status=tx_r485[5];
       system_status_list_1[tx_r485[2]].work_time=tx_r485[7];
	   system_status_list_2[tx_r485[2]].myid=tx_r485[2];
   	   system_status_list_2[tx_r485[2]].size=tx_r485[4];
   	   system_status_list_2[tx_r485[2]].work_status=tx_r485[6];
       system_status_list_2[tx_r485[2]].work_time=tx_r485[8];
	   
		 // LED0=!LED0;
   }


void set_statuslist_1(u8 id,u8 size,u8 work_status,u8 work_time)
{
       system_status_list_1[id].myid=id;
   	   system_status_list_1[id].size=size;
   	   system_status_list_1[id].work_status=work_status;
       system_status_list_1[id].work_time=work_time;

}

void set_statuslist_2(u8 id,u8 size,u8 work_status,u8 work_time)
{
       system_status_list_2[id].myid=id;
   	   system_status_list_2[id].size=size;
   	   system_status_list_2[id].work_status=work_status;
       system_status_list_2[id].work_time=work_time;

}


void offset_idlepower()  //¹¦ÂÊ²¹³¥º¯Êı£¬Èı¸ö²ÎÊı ÎŞ¹¦¹¦ÂÊ ¹¦ÂÊÒòÊı ¿ÕÏĞ¶ÓÁĞ
{
 u8 i,j;
 s8 label_idle1,label_idle2;
  turn_flag=1;
  led_on_off(0);
for(i=1;i<33;i++)inquiry_slave_status(i);
 label_idle1=sort_idlenode_list(sort_idle_list_1,system_status_list_1);
  label_idle2=sort_idlenode_list(sort_idle_list_2,system_status_list_2);

	for(j=1;j<=label_idle1;j++)
		{
                            myled();
		              delay_time(1);
		              if(gonglvshishu>90)break;
		                 else{
                    if(wogong_try>=(sort_idle_list_1[j].size))
			             {    order_trans_rs485(mybox.myid,sort_idle_list_1[j].myid,1,1,1);delay_us(10000);
				//	wugongkvar=wugongkvar-(sort_idle_list_1[j].size)*10;//Ä£Äâ²âÊÔ£¬Êµ¼ÊÇé¿öÓ¦È¥µô¸Ã¾
				wogong_try=wogong_try-sort_idle_list_1[j].size;
                    	               }
		   	                  }
		   }

	
 	if(gonglvshishu<90)
 		{
	for(j=1;j<=label_idle2;j++)
		{
                                      myled();
					  delay_time(1);
		   if(gonglvshishu>90)break;
		   else{
                          if(wogong_try>=(sort_idle_list_2[j].size))
                          	            {
						  order_trans_rs485(mybox.myid,sort_idle_list_2[j].myid,1,2,1);delay_us(10000);
					//    wugongkvar=wugongkvar-(sort_idle_list_2[j].size)*10;//Ä£Äâ²âÊÔ£¬Êµ¼ÊÇé¿öÓ¦¸ÃÈ¥µô¸Ã¾ä
				wogong_try=wogong_try-sort_idle_list_2[j].size;
						  }
		   	 }
	    }

 		}
		led_on_off(1);

}

void turn_power(status_list_node *list_1,status_list_node *list_2)//µ½¹¦ÂÊÒòËØÂú×ãÎÈ¶¨Ìõ¼şºó£¬½øÈëµçÈİÆ÷ÂÖĞİº¯Êı
{
u8 i,j,k,t,q;
s8 label_busy1,label_busy2;
 if(turn_flag==1)
   {
   			led_on_off(0);
   for(i=1;i<33;i++)inquiry_slave_status(i); 
   			led_on_off(1);
   turn_label_idle1=sort_idlenode_list(sort_idle_list_1,list_1);//µÃµ½¿ÕÏĞ¶ÓÁĞ
   turn_label_idle2=sort_idlenode_list(sort_idle_list_2,list_2);
   }  
   turn_flag=0;

if(turn_label_idle1!=0||turn_label_idle2!=0)
{	      led_on_off(0);
   for(i=1;i<33;i++)inquiry_slave_status(i);   			
    label_busy1=sort_busynode_list(sort_busy_list_1,list_1);//Ë¢ĞÂlist_1µÄbusy±íµÄÃ¿¸ö¹¤×÷½ÚµãµÄ¹¤×÷Ê±¼ä
    label_busy2=sort_busynode_list(sort_busy_list_2,list_2);//Ë¢ĞÂlist_2µÄbusy±íµÄÃ¿¸ö¹¤×÷½ÚµãµÄ¹¤×÷Ê±¼ä

	if(turn_label_idle1!=0)
		{
      for(i=1;i<=label_busy1;i++)
	  	{if(sort_busy_list_1[i].work_time>=TIME_OUT)
      	      {  
                for(j=1;j<=turn_label_idle1;j++)
                	{  
                       if(sort_busy_list_1[i].size==sort_idle_list_1[j].size)
                       	{ order_trans_rs485(mybox.myid,sort_idle_list_1[j].myid,1,1,1);delay_us(10000);
					                                           delay_us(100000);//ÊµÑé¿´µÆÑÓÊ±£¬ÕæÊµÇé¿ö×¢µô
                             order_trans_rs485(mybox.myid,sort_busy_list_1[i].myid,1,1,0);delay_us(10000);
						 	{ k=sort_busy_list_1[i].myid;
                                                    t=sort_busy_list_1[i].size;
                                                       sort_busy_list_1[i].work_time=0;
							  for(q=j;q<turn_label_idle1;q++)
							  	{sort_idle_list_1[q].myid=sort_idle_list_1[q+1].myid;
                                                          sort_idle_list_1[q].size=sort_idle_list_1[q+1].size;
							      }
							    sort_idle_list_1[turn_label_idle1].myid=k;
								sort_idle_list_1[turn_label_idle1].size=t;
						     } 
					   }
				    }
	          }

	     }
     }
	
    if(turn_label_idle2!=0)
     { for(i=1;i<=label_busy2;i++)
	  	{if(sort_busy_list_2[i].work_time>=TIME_OUT)
      	      {  
                for(j=1;j<=turn_label_idle2;j++)
                	{  
                       if(sort_busy_list_2[i].size==sort_idle_list_2[j].size)
                       	{ order_trans_rs485(mybox.myid,sort_idle_list_2[j].myid,1,2,1);delay_us(10000);
					   					            delay_us(100000);//ÊµÑé¿´µÆÑÓÊ±£¬ÕæÊµÇé¿ö×¢µô
                             order_trans_rs485(mybox.myid,sort_busy_list_2[i].myid,1,2,0);delay_us(10000);
						  	{ k=sort_busy_list_2[i].myid;
                                                    t=sort_busy_list_2[i].size;
                                                        sort_busy_list_2[i].work_time=0;
							  for(q=j;q<turn_label_idle2;q++)
							  	{sort_idle_list_2[q].myid=sort_idle_list_2[q+1].myid;
                                                          sort_idle_list_2[q].size=sort_idle_list_2[q+1].size;
							        }
							    sort_idle_list_2[turn_label_idle2].myid=k;
								sort_idle_list_2[turn_label_idle2].size=t;
						     }

					   }

				    }
	          }

	     }
     }  
   // if(label_idle1==0&&label_idle2==0)break;//ÎŞ¿ÕÏĞ¶ÓÁĞ 
 led_on_off(1);
   }
}

void unload_power(status_list_node *list_1,status_list_node *list_2)
{
 u8 i,j;
 s8 label_busy1,label_busy2;
  turn_flag=1;
  led_on_off(0);
for(i=1;i<33;i++)inquiry_slave_status(i);
 label_busy2=sort_busynode_list(sort_busy_list_2,list_2);
 label_busy1=sort_busynode_list(sort_busy_list_1,list_1);

	for(j=1;j<=label_busy2;j++)
		{
                                  myled();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
					 delay_time(1);
		   if(gonglvshishu<95)break;
		   else{	order_trans_rs485(mybox.myid,sort_busy_list_2[j].myid,1,2,0);delay_us(10000);}
        }

if(gonglvshishu>95)
 {
	for(j=1;j<=label_busy1;j++)
		{
                          myled();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
		              delay_time(1);
		   if(gonglvshishu<95)break;
		   else {	order_trans_rs485(mybox.myid,sort_busy_list_1[j].myid,1,1,0);delay_us(10000);}
	    }

}
				led_on_off(1);

}
s8 sort_idlenode_list(idle_list *sort_idle_list,status_list_node *list)//¿ÕÏĞÓĞĞò¶ÓÁĞ(°´ÈİÁ¿´óĞ¡ÓÉ´óµ½Ğ¡ÅÅÁĞ£¬·µ»Ø¿ÕÏĞ½Úµã¸öÊı)
{
   u8 i,j=1,k,t,flag=0;
   s8 count=0;
   for(i=1;i<33;i++){sort_idle_list[i].myid=0;sort_idle_list[i].size=0;}
   for(i=1;i<33;i++){
   	              if(list[i].work_status==0&&list[i].size!=0)
                      { sort_idle_list[j].myid=list[i].myid;
				        sort_idle_list[j].size=list[i].size;
					    j++;
						count++;
						if(flag==0)flag=1;
   	              	  }
                    }
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=sort_idle_list[i].size;
	   k=sort_idle_list[i].myid;
	   for(j=i-1;j>=1&&t>sort_idle_list[j].size;j--)
	   	{sort_idle_list[j+1].myid=sort_idle_list[j].myid;
         sort_idle_list[j+1].size=sort_idle_list[j].size;
	    }
	   sort_idle_list[j+1].myid=k;
	   sort_idle_list[j+1].size=t;

      }
   	}
    return count;
}
/*
void sort_timenode_list(time_list *sort_time_list,status_list_node *list)//Ê±¼äÓĞĞò¶ÓÁĞ(°´¹¤×÷Ê±¼äÓÉ´óµ½Ğ¡ÅÅÁĞ)
{
u8 i,j=1,k,t,s;
for(i=1;i<33;i++){sort_time_list[i].myid=0;sort_time_list[i].work_time=0;sort_time_list[i].size=0;}
 for(i=1;i<33;i++){
   	              if(list[i].myid!=0)
                     {sort_time_list[j].myid=list[i].myid;
				      sort_time_list[j].work_time=list[i].work_time;
					  sort_time_list[j].size=list[i].size;
					  j++;
   	              	 }
                    }
   for(i=2;i<33;i++)
   	{
       t=sort_time_list[i].work_time;
	   k=sort_time_list[i].myid;
	   s=sort_time_list[i].size;
	   for(j=i-1;j>=1&&t>sort_time_list[j].work_time;j--)
	   	{sort_time_list[j+1].myid=sort_time_list[j].myid;
         sort_time_list[j+1].work_time=sort_time_list[j].work_time;
		 sort_time_list[j+1].size=sort_time_list[j].size;
	    }
	   sort_time_list[j+1].myid=k;
	   sort_time_list[j+1].work_time=t;
       sort_time_list[j+1].size=s;
   }
}
*/
s8 sort_busynode_list(busy_list *sort_busy_list,status_list_node *list)//Ã¦ÂµÓĞĞò¶ÓÁĞ(°´ÈİÁ¿´óĞ¡ÓÉĞ¡µ½´óÅÅÁĞ)
{
   u8 i,j=1,k,t,w,flag=0;
   s8 count=0;
   for(i=1;i<33;i++){sort_busy_list[i].myid=0;sort_busy_list[i].size=0;}
   for(i=1;i<33;i++){
   	              if(list[i].work_status==1&&list[i].size!=0)
                      { sort_busy_list[j].myid=list[i].myid;
				        sort_busy_list[j].size=list[i].size;
						sort_busy_list[j].work_time=list[i].work_time;
					    j++;
						if(flag==0)flag=1;
						count++;
   	              	  }
                    }
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=sort_busy_list[i].size;
	   k=sort_busy_list[i].myid;
	   w=sort_busy_list[i].work_time;
	   for(j=i-1;j>=1&&t<sort_busy_list[j].size;j--)
	   	{sort_busy_list[j+1].myid=sort_busy_list[j].myid;
         sort_busy_list[j+1].size=sort_busy_list[j].size;
		 sort_busy_list[j+1].work_time=sort_busy_list[j].work_time;
	    }
	   sort_busy_list[j+1].myid=k;
	   sort_busy_list[j+1].size=t;
       sort_busy_list[j+1].work_time=w;
     }
   	}
  return count;
}




void delay_time(u32 time)
{ heartbeat(time);
}  //±¾ÏµÍ³µÄÑÓÊ±º¯Êı£¬time*10ms

void inquiry_slave_status(u8 id)   
  {  u8 *msg;
        u8 err;
   order_trans_rs485(mybox.myid,id,2,0,0);
  // delay_us(10000);
   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/25,&err);
   if(err==OS_ERR_TIMEOUT){set_statuslist_1(id,0,2,0);set_statuslist_2(id,0,2,0);}//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
	  rs485_trans_status(msg);
	if(id==mybox.myid)
		{
	   set_statuslist_1(mystatus.myid,mystatus.size[0],mystatus.work_status[0],mystatus.work_time[0]);//Ö÷»ú×´Ì¬ĞÅÏ¢Ğ´Èë×´Ì¬±í
       set_statuslist_2(mystatus.myid,mystatus.size[1],mystatus.work_status[1],mystatus.work_time[1]);
		}
	} //²éÑ¯´Ó»ú×´Ì¬²¢±£´æµ½´Ó»ú×´Ì¬±íÖĞ£¬²ÎÊıidÊÇÒª²éÑ¯µÄ´Ó»úºÅ






/*******************¹¦ÂÊÒòËØÏà¹Øº¯Êı*****************************/

u16 power_computer()
{
        u16 i;
		u32 tempa=0,tempb=0;
		u16 adc_vx=0,adc_vmax=0,adc_ix=0,adc_imax=0;
		u8 phase_zhi=0;
		 float temp=0;

		id_num=AT24CXX_ReadOneByte(0x0010);
		key_idset();

		 for(i=0;i<120;i++)
	  	 {
	  	 adc_vx=Get_Adc_Average(ADC_Channel_1,10);
		  // adc_vx=Get_Adc(ADC_Channel_1);
		   if(adc_vx>adc_vmax)
		   adc_vmax=adc_vx;
	  	 }
	   for(i=0;i<120;i++)
	  {
		 adc_ix=Get_Adc_Average(ADC_CH4,10);
		 if(adc_ix>adc_imax)
		 adc_imax=adc_ix;
	  }
	  temp=(float)adc_vmax*(3.3/4096);
	 dianya_zhi=(u16)(518*temp-660);
	  temp=(float)adc_imax*(3.3/4096);
	  dianliuzhi=(u32)(60*temp-80);
	  adc_vmax=0;
	  adc_imax=0;
	  if(TIM3CH1_CAPTURE_STA&0X80)//Íê³ÉÒ»´Î²É¼¯
		{
			tempa=TIM3CH1_CAPTURE_STA&0X3F;
			tempa*=65536;					//Òç³öÊ±¼ä×ÜºÍ
			tempa+=TIM3CH1_CAPTURE_VAL;		//µÃµ½TI1¶ËĞÅºÅÖÜÆÚÊ±¼ä


			tempb=TIM3CH1_CAPTURE_STA&0X3F;
			tempb*=65536;					//Òç³öÊ±¼ä×ÜºÍ
			tempb+=TIM3CH1_CAPTURE_PHA;		//µÃµ½TI2 TI1ÉÏÉıÑØÊ±¼ä²îÖµ¼´ÏàÎ»²îÊ±¼ä

	 		 if(tempb<=5000)			   //¸ĞĞÔ¸ºÔØÕı½Ó
			 {
			 	phase_zhi=tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			  if((10000<=tempb)&&(tempb<=15000))			 //¸ĞĞÔ¸ºÔØ·´½Ó
			 {
			 	phase_zhi=((tempb*360)/tempa)-180;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 if((5000<tempb)&&(tempb<10000))	   //ÈİĞÔ¸ºÔØÕı½Ó
			 {
				/*ÏÔÊ¾ÈİĞÔ¹¦ÂÊ·ûºÅ*/
			 	phase_zhi=180-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 if((15000<tempb)&&(tempb<20000))	   //ÈİĞÔ¸ºÔØ·´½Ó
			 {
				/*ÏÔÊ¾ÈİĞÔ¹¦ÂÊ·ûºÅ*/
			 	phase_zhi=360-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
//			 wugongkvar=(uint16_t)(1.732*dianliuzhi*dianya_zhi*k*co[phase_zhi]);
			
			TIM3CH1_CAPTURE_STA=0;			//¿ªÆôÏÂÒ»´Î²¶»ñ
			return gonglvshishu;
		}
	  return gonglvshishu;

//ÎŞ¹¦¹¦ÂÊ
}

void gonglvyinshu()
{
        u16 i;
		u32 tempa,tempb;
		u16 adc_vx,adc_vmax=0,adc_ix,adc_imax=0;
		u8 phase_zhi;
		 float temp;

		id_num=AT24CXX_ReadOneByte(0x0010);
	//	key_idset();

		 for(i=0;i<120;i++)
	  	 {
	  	 adc_vx=Get_Adc_Average(ADC_Channel_1,10);
		  // adc_vx=Get_Adc(ADC_Channel_1);
		   if(adc_vx>adc_vmax)
		   adc_vmax=adc_vx;
	  	 }
	   for(i=0;i<120;i++)
	  {
		 adc_ix=Get_Adc_Average(ADC_CH4,10);
		 if(adc_ix>adc_imax)
		 adc_imax=adc_ix;
	  }
	  temp=(float)adc_vmax*(3.3/4096);
	 dianya_zhi=(u16)(518*temp-663);
	  temp=(float)adc_imax*(3.3/4096);
	  dianliuzhi=(u32)(60*temp-74);
	  adc_vmax=0;
	  adc_imax=0;
	  if(TIM3CH1_CAPTURE_STA&0X80)//Íê³ÉÒ»´Î²É¼¯
		{
			tempa=TIM3CH1_CAPTURE_STA&0X3F;
			tempa*=65536;					//Òç³öÊ±¼ä×ÜºÍ
			tempa+=TIM3CH1_CAPTURE_VAL;		//µÃµ½TI1¶ËĞÅºÅÖÜÆÚÊ±¼ä


			tempb=TIM3CH1_CAPTURE_STA&0X3F;
			tempb*=65536;					//Òç³öÊ±¼ä×ÜºÍ
			tempb+=TIM3CH1_CAPTURE_PHA;		//µÃµ½TI2 TI1ÉÏÉıÑØÊ±¼ä²îÖµ¼´ÏàÎ»²îÊ±¼ä

	 		 if(tempb<=5000)			   //¸ĞĞÔ¸ºÔØÕı½Ó
			 {
			 	phase_zhi=tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			  if((10000<=tempb)&&(tempb<=15000))			 //¸ĞĞÔ¸ºÔØ·´½Ó
			 {
			 	phase_zhi=((tempb*360)/tempa)-180;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 if((5000<tempb)&&(tempb<10000))	   //ÈİĞÔ¸ºÔØÕı½Ó
			 {
				/*ÏÔÊ¾ÈİĞÔ¹¦ÂÊ·ûºÅ*/
			 	phase_zhi=180-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 if((15000<tempb)&&(tempb<20000))	   //ÈİĞÔ¸ºÔØ·´½Ó
			 {
				/*ÏÔÊ¾ÈİĞÔ¹¦ÂÊ·ûºÅ*/
			 	phase_zhi=360-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 wugongkvar=(uint16_t)((1.732*dianliuzhi*dianya_zhi*k*co[phase_zhi])/100000);
			
			TIM3CH1_CAPTURE_STA=0;			//¿ªÆôÏÂÒ»´Î²¶»ñ
		
		}

//ÎŞ¹¦¹¦ÂÊ
}	 


void myled()
  {
gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
key_idset();//°´¼üÓëÏÔÊ¾¹¦ÄÜ
delay_ms(50);//Ã»ÓĞÑÓÊ±£¬ÆÁ»áËÀ»ú
}


	 


