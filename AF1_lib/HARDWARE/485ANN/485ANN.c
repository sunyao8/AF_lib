#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "485ANN.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "key.h"
#include "math.h"
/***********************************************************************/
u16 RS485_RX_BUF[64]; 		//½ÓÊÕ»º³å,×î´ó64¸ö×Ö½Ú
//½ÓÊÕµ½µÄÊý¾Ý³¤¶È
u8 RS485_RX_CNT=0;  
//Ä£Ê½¿ØÖÆ
 u16  dog_clock=2;

 OS_EVENT * RS485_MBOX,* RS485_STUTAS_MBOX;			//	rs485ÓÊÏäÐÅºÅÁ¿
 OS_EVENT *Heartbeat;			 //ÐÄÌøÐÅºÅÁ¿
OS_EVENT *master_led_task;
OS_EVENT * sub_machine1_open;		//ÏÂÎ»»úÃüÁîÐÅºÅ
OS_EVENT * sub_machine1_close;		//ÏÂÎ»»úÃüÁîÐÅºÅ
OS_EVENT * sub_machine2_open;		//ÏÂÎ»»úÃüÁîÐÅºÅ
OS_EVENT * sub_machine2_close;		//ÏÂÎ»»úÃüÁîÐÅºÅ

OS_EVENT *scan_slave;

u8 cont=0;//ÓÃÓÚ¸ü¸ÄÖ÷»úºÅµÄ¼Ç´ÎÊýÆ÷
u32 life_time_1=0;//´Ó»ú1¹¤×÷Ê±¼äµÄÖÐ¼ä±äÁ¿
u32 life_time_2=0;//´Ó»ú2¹¤×÷Ê±¼äµÄÖÐ¼ä±äÁ¿
u32 idle_time=0;//Ö÷»úÓÃÓÚÂÖÐÝµÄÊ±¼ä
u32 scan_time=0;//Ö÷»úÓÃÓÚ´ÓÐÂ±éÀú´Ó»úµÄµÄÊ±¼ä
u16 m1_opentime,m2_opentime,m1_closetime,m2_closetime;//´Ó»úÊ¹ÓÃ±äÁ¿
u8 true_worktime1_flag=0,true_worktime2_flag=0;//´Ó»úÊ¹ÓÃ±äÁ¿£¬ÓÃÓÚ±êÊ¶ÏÂÎ»»úÕæÕýÍ¸ÇÐµÄÊ±¼äµã
u8 turn_flag=1;//ÂÖÐÝÊ¹ÓÃ±äÁ¿
s8 turn_label_idle=0;//ÂÖÐÝÊ¹ÓÃ±äÁ¿

box mybox;
status_box mystatus;
idle_list sort_idle_list_1[33];
idle_list sort_idle_list_2[33];
turn_node turn_idle_list[65];
busy_list sort_busy_list_1[33];
busy_list sort_busy_list_2[33];

status_list_node system_status_list_1[33];
status_list_node system_status_list_2[33];

u8 idle_done_nodelist_1[33];
u8 idle_done_nodelist_2[33];
u8 done_list1_flag=0,done_list2_flag=0;
u8 done_count_1=0,done_count_2=0;

u8 slave[33];

//u8 rs485buf[LEN_control];//·¢ËÍ¿ØÖÆÐÅÏ¢
u8 rs485buf[LEN_control];//·¢ËÍ¿ØÖÆÐÅÏ¢
u8 statusbuf[LEN_status];//·¢ËÍ×´Ì¬ÐÅÏ¢


u8 alarm_lock=0;
/****************************************************************/
u8 si[]={0,0,0,5,7,9,10,12,14,16,        //0~9
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
          16,14,12,10,9,7,5,0,0,0			  //80~90
		  };


/**********************²âÊÔÎÞ¹¦¹¦ÂÊ Êý¾Ý*************************************/


/************************************************************/
u16 wugong_95,wugong_computer;

extern u8 id_num;
extern u8 grafnum,tempshuzhi,gonglvshishu;
extern u16 dianya_zhi,	wugongkvar,k;
extern u32	dianliuzhi;
s8 L_C_flag;//¸ÐÐÔÈÝÐÔ±ê×¼±äÁ¿
/*****************************************************/
 void TIM4_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //Ê±ÖÓÊ¹ÄÜ
	
	//¶¨Ê±Æ÷TIM4³õÊ¼»¯
	TIM_TimeBaseStructure.TIM_Period = arr; //ÉèÖÃÔÚÏÂÒ»¸ö¸üÐÂÊÂ¼þ×°Èë»î¶¯µÄ×Ô¶¯ÖØ×°ÔØ¼Ä´æÆ÷ÖÜÆÚµÄÖµ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //ÉèÖÃÓÃÀ´×÷ÎªTIMxÊ±ÖÓÆµÂÊ³ýÊýµÄÔ¤·ÖÆµÖµ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ÉèÖÃÊ±ÖÓ·Ö¸î:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIMÏòÉÏ¼ÆÊýÄ£Ê½
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯TIMxµÄÊ±¼ä»ùÊýµ¥Î»
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //Ê¹ÄÜÖ¸¶¨µÄTIM4ÖÐ¶Ï,ÔÊÐí¸üÐÂÖÐ¶Ï

	//ÖÐ¶ÏÓÅÏÈ¼¶NVICÉèÖÃ
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //ÏÈÕ¼ÓÅÏÈ¼¶0¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //´ÓÓÅÏÈ¼¶3¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQÍ¨µÀ±»Ê¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);  //³õÊ¼»¯NVIC¼Ä´æÆ÷


	TIM_Cmd(TIM4, ENABLE);  //Ê¹ÄÜTIMx					 
}
 
 void TIM4_IRQHandler(void)   //TIM4ÖÐ¶Ï
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //¼ì²éTIM4¸üÐÂÖÐ¶Ï·¢ÉúÓë·ñ
		{	  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //Çå³ýTIMx¸üÐÂÖÐ¶Ï±êÖ¾
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
		 if (mystatus.work_status[0]==1&&true_worktime1_flag==1)  //¹¤×÷Ê±¼äµÄ¼ÆÊ±
		    {  
		        life_time_1++;
		       if(life_time_1==5)
			   { mystatus.work_time[0]++;
			     life_time_1=0;
				  }
			        if(mystatus.work_time[0]==37)mystatus.work_time[0]=39;
				  if(mystatus.work_time[0]==254)mystatus.work_time[0]=0;
			   }
		 	
		 if(mystatus.work_status[1]==1&&true_worktime2_flag==1)
		 	{ 
		 	life_time_2++;
              if(life_time_2==5)
			  	{  mystatus.work_time[1]++;
			       life_time_2=0;
              	         }   
                   if(mystatus.work_time[1]==37)mystatus.work_time[1]=39;
			 if(mystatus.work_time[1]==254)mystatus.work_time[1]=0;	   
			    
		     }
		   if (mystatus.work_status[0]==0&&true_worktime1_flag==0)  //¹¤×÷Ê±¼äÇåÁã
		    {   mystatus.work_time[0]=0;
		   }
		    if (mystatus.work_status[1]==0&&true_worktime2_flag==0)  //¹¤×÷Ê±¼äÇåÁã
		    {   mystatus.work_time[1]=0;
			}

			if(mybox.master==1)
		       {  idle_time++;
			if(idle_time==65535)idle_time=0;
			}

			if(mybox.master==1)	
				{
                                   scan_time++;
					if(scan_time==65530)scan_time=0;			   

			       }
	/*		
			if(m1_close==1)
				{
                            m1_opentime=0;
				m1_closetime++;
			       if(m1_closetime==65530)m1_closetime=0;
			        }
			if(m1_close==0)
				{
				m1_closetime=0;
				m1_opentime++;
			       if(m1_opentime==65530)m1_opentime=0;

			       }
				if(m2_close==1)
				{
                            m2_opentime=0;
				m2_closetime++;
			       if(m2_closetime==65530)m2_closetime=0;
			        }
			if(m2_close==0)
				{
				m2_closetime=0;
				m2_opentime++;
			       if(m2_opentime==65530)m2_opentime=0;

			       }
           */


	}
   	OSIntExit();  
}
//////////////////////////////////////////////////////////////////////////////////
void TIM3_Cap_Init(u16 arr,u16 psc)
{	NVIC_InitTypeDef NVIC_InitStructure;	 
	RCC->APB1ENR|=1<<1;   	//TIM3 Ê±ÖÓÊ¹ÄÜ 
	RCC->APB2ENR|=1<<3;    	//Ê¹ÄÜPORTBÊ±ÖÓ  
	 
	GPIOB->CRL&=0XFFFFFF00;	//PB0 PB1Çå³ýÖ®Ç°ÉèÖÃ  
	GPIOB->CRL|=0X00000088;	//PB0 PB1ÊäÈë   
	GPIOB->ODR&=~(1<<0);		//PB0 PB1ÏÂÀ­
	GPIOB->ODR&=~(1<<1);
	  
 	TIM3->ARR=arr;  		//Éè¶¨¼ÆÊýÆ÷×Ô¶¯ÖØ×°Öµ   
	TIM3->PSC=psc;  		//Ô¤·ÖÆµÆ÷ 

	TIM3->CCMR2|=1<<0;		//CC3S=01 	Ñ¡ÔñÊäÈë¶Ë IC3Ó³Éäµ½TI3ÉÏ
 	TIM3->CCMR2|=0<<4; 		//IC3F=0000 ÅäÖÃÊäÈëÂË²¨Æ÷ ²»ÂË²¨
 	TIM3->CCMR2|=0<<2;  	//IC3PS=00 	ÅäÖÃÊäÈë·ÖÆµ,²»·ÖÆµ 

	///TIM3->CCER|=0<<9; 		//CC3P=0	ÉÏÉýÑØ²¶»ñ1
	TIM3->CCER|=1<<9;                //CC3P=0	ÏÂÉýÑØ²¶»ñ1
	TIM3->CCER|=1<<8; 		//CC3E=1 	ÔÊÐí²¶»ñ¼ÆÊýÆ÷1µÄÖµµ½²¶»ñ¼Ä´æÆ÷ÖÐ

	TIM3->DIER|=1<<3;   	//ÔÊÐí²¶»ñ3ÖÐ¶Ï				
	TIM3->DIER|=1<<0;   	//ÔÊÐí¸üÐÂ1ÖÐ¶Ï	

	TIM3->CCMR2|=1<<8;		//CC4S=01 	Ñ¡ÔñÊäÈë¶Ë IC4Ó³Éäµ½TI4ÉÏ
 	TIM3->CCMR2|=0<<12; 	//IC4F=0000 ÅäÖÃÊäÈëÂË²¨Æ÷ ²»ÂË²¨
 	TIM3->CCMR2|=0<<10; 	//IC4PS=00 	ÅäÖÃÊäÈë·ÖÆµ,²»·ÖÆµ 

///	TIM3->CCER|=0<<13; 		//CC4P=0	ÉÏÉýÑØ²¶»ñ
TIM3->CCER|=1<<13;                      //CC4P=0	ÏÂÉýÑØ²¶»ñ1
	TIM3->CCER|=1<<12; 		//CC4E=1 	ÔÊÐí²¶»ñ¼ÆÊýÆ÷µÄÖµµ½²¶»ñ¼Ä´æÆ÷ÖÐ

	TIM3->DIER|=1<<4;   	//ÔÊÐí²¶»ñ4ÖÐ¶Ï				
	TIM3->DIER|=1<<0;   	//ÔÊÐí¸üÐÂÖÐ¶Ï
	TIM3->CR1|=0x01;    	//Ê¹ÄÜ¶¨Ê±Æ÷2
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //ÏÈÕ¼ÓÅÏÈ¼¶0¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  //´ÓÓÅÏÈ¼¶4¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQÍ¨µÀ±»Ê¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);  //³õÊ¼»¯NVIC¼Ä´æÆ÷
	   
}

//²¶»ñ×´Ì¬
//[7]:0,Ã»ÓÐ³É¹¦µÄ²¶»ñ;1,³É¹¦²¶»ñµ½Ò»´Î.
//[6]:0,»¹Ã»²¶»ñµ½¸ßµçÆ½;1,ÒÑ¾­²¶»ñµ½¸ßµçÆ½ÁË.
//[5:0]:²¶»ñ¸ßµçÆ½ºóÒç³öµÄ´ÎÊý
u8  TIM3CH1_CAPTURE_STA=0;	//ÊäÈë²¶»ñ×´Ì¬		    				
u16	TIM3CH1_CAPTURE_VAL;	//ÊäÈë²¶»ñÖµ
u16	TIM3CH1_CAPTURE_PHA;	//ÊäÈë²¶»ñÖµ
//¶¨Ê±Æ÷3ÖÐ¶Ï·þÎñ³ÌÐò	 
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
		if(tsr&0x10)//²¶»ñ1·¢Éú²¶»ñÊÂ¼þ
		{	
			if(TIM3CH1_CAPTURE_STA&0X40)		//²¶»ñµ½Ò»¸öÏÂ½µÑØ 		
			{	  			
				TIM3CH1_CAPTURE_STA|=0X80;		//±ê¼Ç³É¹¦²¶»ñµ½Ò»´Î¸ßµçÆ½Âö¿í
			    TIM3CH1_CAPTURE_VAL=TIM3->CCR4;	//»ñÈ¡µ±Ç°µÄ²¶»ñÖµ.
	 		///	TIM3->CCER&=~(1<<1);			//CC1P=0 ÉèÖÃÎªÉÏÉýÑØ²¶»ñ
			}else  								//»¹Î´¿ªÊ¼,µÚÒ»´Î²¶»ñÉÏÉýÑØ
			{
				TIM3CH1_CAPTURE_STA=0;			//Çå¿Õ
				TIM3CH1_CAPTURE_VAL=0;
				TIM3CH1_CAPTURE_STA|=0X40;		//±ê¼Ç²¶»ñµ½ÁËÉÏÉýÑØ
	 			TIM3->CNT=0;					//¼ÆÊýÆ÷Çå¿Õ
			  ///	TIM3->CCER&=~(1<<1);			//CC1P=0 ÉèÖÃÎªÉÏÉýÑØ²¶»ñ
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
	TIM3->SR=0;//Çå³ýÖÐ¶Ï±êÖ¾Î» 	    
}


//////////////////////////////////////////////////////////
		void USART2_IRQHandler(void)
{
 
	u8 RS485_RX_BUF[64];
	#ifdef OS_TICKS_PER_SEC	 	//Èç¹ûÊ±ÖÓ½ÚÅÄÊý¶¨ÒåÁË,ËµÃ÷ÒªÊ¹ÓÃucosIIÁË.
	OSIntEnter();    
      #endif
 		
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //½ÓÊÕµ½Êý¾Ý
	{	 
	 			 
		 RS485_RX_BUF[RS485_RX_CNT++]=USART_ReceiveData(USART2); 	//¶ÁÈ¡½ÓÊÕµ½µÄÊý¾Ý
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='&'){RS485_RX_BUF[0]='&'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='*')
		{
				RS485_RX_CNT=0;

				
				if(RS485_RX_BUF[1]=='#'){OSMboxPost(RS485_STUTAS_MBOX,(void*)&RS485_RX_BUF);}
				  else OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);
		} 
	}  	
	#ifdef OS_TICKS_PER_SEC	 	//Èç¹ûÊ±ÖÓ½ÚÅÄÊý¶¨ÒåÁË,ËµÃ÷ÒªÊ¹ÓÃucosIIÁË.
	OSIntExit();  											 
#endif

} 

void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			//ÉèÖÃÎª·¢ËÍÄ£Ê½
  	for(t=0;t<len;t++)		//Ñ­»··¢ËÍÊý¾Ý
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);		
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;				//ÉèÖÃÎª½ÓÊÕÄ£Ê½	

}

void initmybox()//³õÊ¼»¯×ÔÉíÐÅÏ¢
{  	 
  
  mybox.master=0;
 mybox.start='&';
mybox.myid=AT24CXX_ReadOneByte(0x0010);
///mybox.myid=1;
 mybox.source=0;
 mybox.destination=0;
 mybox.send=0;
 mybox.relay=0;
 mybox.message=0;
 mybox.end='*';	
						
}

void turn_master_id(u8 id)//¸Ä±äµ±Ç°Õû¸öÏµÍ³ÖÐÖ÷»úµÄIDºÅ
{
   u8 flag=0;
	{ 
	  flag=cont;
      if(id==(flag)){
	delay_time(2);
         mybox.master=1;
	    OSTaskResume(MASTER_TASK_PRIO);
		cont=1;
	  }
	 //  LED1=!LED1;
	  
      }
   }






 int rs485_trans_order(u8 *tx_r485)//½âÎöÓÉÖ÷»ú·¢ËÍ¹ýÀ´µÄÐÅºÅ£¬²¢·¢ËÍ¸øÏÂÎ»»ú
{ 
  dianya_zhi=comp_16(tx_r485[6],tx_r485[7]);
  dianliuzhi=comp_16(tx_r485[8],tx_r485[9]);
  wugongkvar=comp_16(tx_r485[10],tx_r485[11]);
  //tempshuzhi=tx_r485[12];
  gonglvshishu=tx_r485[12];
   if(mybox.myid==tx_r485[2]||tx_r485[2]==0)//ÅÐ¶ÏÊÇ·ñÊÇ·¢¸ø±¾»úµÄÐÅÏ¢»òÊÇ¹ã²¥ÐÅÏ¢
   	{
   	 mybox.source=tx_r485[1];
   	 mybox.send=tx_r485[3];
     mybox.relay=tx_r485[4];
     mybox.message=tx_r485[5];
     return 1;
   	}
   else return 0;
}

 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message)//Ö÷»ú³ÌÐò£¬Ö÷»úÃüÁî½âÎö³ÉRS485ÐÅÏ¢£¬·¢ËÍ¸øÄ¿µÄ´Ó»ú
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
      rs485buf[0]='&';//Ð­ÒéÍ·
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
	//rs485buf[12]=tempshuzhi;
	rs485buf[12]=gonglvshishu;
	rs485buf[13]='*';//Ð­ÒéÎ²
	RS485_Send_Data(rs485buf,14);//·¢ËÍ5¸ö×Ö½Ú
	if(destination==source){mybox.send=send;slave_control(relay, message);}//Èç¹ûÐÅÏ¢·¢¸øµÄ×Ô¼º
	OS_EXIT_CRITICAL();	
}

u16 comp_16(u16 a,u16 b)
{
u16 value=0;
value=((a&0x00FF)+((b<<8)&0xFF00));
return value;
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










void led_on_off(u8 on_off,u8 j) //²ÎÊýÖµ1 Îª´ò¿ªled £¬0Îª¹Ø±Õled
{
u8 i;
if(on_off==ALL_NODE_LCD_UNLOCK)
    {
        for(i=1;i<10;i++)//65
	   { order_trans_rs485(mybox.myid,0,ALL_NODE_LCD_UNLOCK,0,0);
        gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
       temperature();
        key_lcd();

         
          }	 
    }
if(on_off==ALL_NODE_LCD_LOCK)
    {
        for(i=1;i<20;i++) //65
	   { order_trans_rs485(mybox.myid,0,ALL_NODE_LCD_LOCK,0,0);
            gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
temperature();
key_lcd();

          }	 
    }

if(on_off==IDLE_NODE_LCD_LOCK)
    {
        for(i=1;i<20;i++) //65
	   { order_trans_rs485(mybox.myid,0,IDLE_NODE_LCD_LOCK,0,0);
            gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
temperature();
key_lcd();

          }	 
    }


if(on_off==BUSY_NODE_LCD_LCOK)
    {
        for(i=1;i<20;i++) //65
	   { order_trans_rs485(mybox.myid,0,BUSY_NODE_LCD_LCOK,0,0);
            // delay_us(10000);
            gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
temperature();
key_lcd();

          }	 
    }

if(on_off==NODE_LCD_LOCK_BASE)
    {
        for(i=1;i<20;i++) //65
	   { order_trans_rs485(mybox.myid,0,NODE_LCD_LOCK_BASE+j,0,0);
            gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
        temperature();
          key_lcd();

          }	 
    }



}


/*****************************»ØÀ¡ÐÅÏ¢º¯Êý********************************************/


void init_mystatus(u8 size_1,u8 size_2,u8 work_status_1,u8 work_status_2,u8 work_time_1,u8 work_time_2)
{
mystatus.myid= mybox.myid;
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



 void status_trans_rs485(status_box *mystatus)//´Ó»ú³ÌÐò
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


 void rs485_trans_status(u8 *tx_r485)//Ö÷»ú³ÌÐò£¬Ö÷»úÃüÁî½âÎö³ÉRS485ÐÅÏ¢£¬·¢ËÍ¸øÄ¿µÄ´Ó»ú
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


void offset_idlepower()  //¹¦ÂÊ²¹³¥º¯Êý£¬Èý¸ö²ÎÊý ÎÞ¹¦¹¦ÂÊ ¹¦ÂÊÒòÊý ¿ÕÏÐ¶ÓÁÐ
{
 u8 i,j;
 u8 label_idle1,label_idle2;
  turn_flag=1;
  led_on_off(IDLE_NODE_LCD_LOCK,0);
for(i=1;i<=slave[0];i++)inquiry_slave_status(slave[i]);
 label_idle1=sort_idlenode_list(sort_idle_list_1,system_status_list_1);
  label_idle2=sort_idlenode_list(sort_idle_list_2,system_status_list_2);
 
/***********************************************************************/
//label_idle1=0;//ÊµÑé Ö»Í¶ÇÐµÚ¶þ×éÓÃ £¬²úÆ·±ØÐëÈ¥µôÕâ¾ä»°
/************************************************************************/


	for(j=1;j<=label_idle1;j++)
		{
                         if(done_list1_flag==0)
                            {

				   for(i=1;i<=label_idle1;i++)
				   	    {
                                          idle_done_nodelist_1[sort_idle_list_1[i].myid]=0;

				           }
				   done_list1_flag=1;
			       }			 
			delay_time(1);
				myled();  //120ms
		
		              if(gonglvshishu>90)break;
		                 else{
                    if((wugong_computer)>=(sort_idle_list_1[j].size))
			             {    if(idle_done_nodelist_1[sort_idle_list_1[j].myid]==0)
			                   {                   
                                           led_on_off(NODE_LCD_LOCK_BASE,sort_idle_list_1[j].myid);//1000ms
					order_trans_rs485(mybox.myid,sort_idle_list_1[j].myid,Sub_Order,1,1);
							   delay_ms(13000);
                                          idle_done_nodelist_1[sort_idle_list_1[j].myid]=1;
										 
		
					    }
					}
					done_count_1++;
					if(done_count_1==label_idle1)
						{done_count_1=0;done_list1_flag=0;}
		   	                  }
		   }

	
 	if(gonglvshishu<90)
 		{
	for(j=1;j<=label_idle2;j++)
		{
		                         if(done_list2_flag==0)
                            {

				   for(i=1;i<=label_idle2;i++)
				   	    {
                                          idle_done_nodelist_2[sort_idle_list_2[i].myid]=0;

				           }
				   done_list2_flag=1;
			       }
		 
			delay_time(1);
				myled();
		

		   if(gonglvshishu>90)break;
		   else{
                          if((wugong_computer)>=(sort_idle_list_2[j].size))
                          	            {  if(idle_done_nodelist_2[sort_idle_list_2[j].myid]==0)
                          	                {     
                                           led_on_off(NODE_LCD_LOCK_BASE,sort_idle_list_1[j].myid);
						  order_trans_rs485(mybox.myid,sort_idle_list_2[j].myid,Sub_Order,2,1);
						  delay_ms(13000);
                                                 idle_done_nodelist_2[sort_idle_list_2[j].myid]=1;
										
                          	                }
						  done_count_2++;
						  	if(done_count_2==label_idle2)
						  		{done_count_2=0;done_list2_flag=0;}
						  
						  }
		   	 }
	    }

 		}

}

void turn_power(status_list_node *list_1,status_list_node *list_2)//µ½¹¦ÂÊÒòËØÂú×ãÎÈ¶¨Ìõ¼þºó£¬½øÈëµçÈÝÆ÷ÂÖÐÝº¯Êý
{
u8 i,j,k,t,q;
u8 label_busy1,label_busy2;
 if(turn_flag==1)
   {
   			led_on_off(IDLE_NODE_LCD_LOCK,0);
   for(i=1;i<=slave[0];i++)inquiry_slave_status(slave[i]); 
   turn_label_idle=turn_idlenode_list(turn_idle_list,list_1,list_2);//µÃµ½¿ÕÏÐ¶ÓÁÐ
   }  
   turn_flag=0;

if(turn_label_idle!=0)
{	      led_on_off(BUSY_NODE_LCD_LCOK,0);
   for(i=1;i<=slave[0];i++)inquiry_slave_status(slave[i]);   			
    label_busy1=sort_busynode_list(sort_busy_list_1,list_1);//Ë¢ÐÂlist_1µÄbusy±íµÄÃ¿¸ö¹¤×÷½ÚµãµÄ¹¤×÷Ê±¼ä
    label_busy2=sort_busynode_list(sort_busy_list_2,list_2);//Ë¢ÐÂlist_2µÄbusy±íµÄÃ¿¸ö¹¤×÷½ÚµãµÄ¹¤×÷Ê±¼ä

      for(i=1;i<=label_busy1;i++)
	  	{ if(sort_busy_list_1[i].work_time>=TIME_OUT)
      	      {  
				for(j=1;j<=turn_label_idle;j++)
                	{  
                       if(sort_busy_list_1[i].size==turn_idle_list[j].size)
                       	{ order_trans_rs485(mybox.myid,turn_idle_list[j].myid,Sub_Order+1,turn_idle_list[j].group,1);delay_us(10000);
                             order_trans_rs485(mybox.myid,sort_busy_list_1[i].myid,Sub_Order+2,1,0);
						 	{ k=sort_busy_list_1[i].myid;
                                                    t=sort_busy_list_1[i].size;
                                                     //  sort_busy_list_1[i].work_time=0;
							  for(q=j;q<turn_label_idle;q++)
							  	{turn_idle_list[q].myid=turn_idle_list[q+1].myid;
                                                          turn_idle_list[q].size=turn_idle_list[q+1].size;
								turn_idle_list[q].group=turn_idle_list[q+1].group;
							      }
							    turn_idle_list[turn_label_idle].myid=k;
							    turn_idle_list[turn_label_idle].size=t;
							   turn_idle_list[turn_label_idle].group=1;
						     } 
							 delay_ms(30200);
                                           break;//·ÀÖ¹±¾Ñ­»·ÖÐ¶Ôi£¬ÖØ¸´Æ¥ÅäÍ¶ÇÐ

					   }
				    }
      	}
      	}

	      for(i=1;i<=label_busy2;i++)	
   { if(sort_busy_list_2[i].work_time>=TIME_OUT)
      	      {  
				for(j=1;j<=turn_label_idle;j++)
                	{  
                       if(sort_busy_list_2[i].size==turn_idle_list[j].size)
                       	{ order_trans_rs485(mybox.myid,turn_idle_list[j].myid,Sub_Order+1,turn_idle_list[j].group,1);delay_us(10000);
                             order_trans_rs485(mybox.myid,sort_busy_list_2[i].myid,Sub_Order+2,2,0);delay_us(10000);
						 	{ k=sort_busy_list_2[i].myid;
                                                    t=sort_busy_list_2[i].size;
                                                     //  sort_busy_list_2[i].work_time=0;
							  for(q=j;q<turn_label_idle;q++)
							  	{turn_idle_list[q].myid=turn_idle_list[q+1].myid;
                                                          turn_idle_list[q].size=turn_idle_list[q+1].size;
								turn_idle_list[q].group=turn_idle_list[q+1].group;
							      }
							    turn_idle_list[turn_label_idle].myid=k;
							    turn_idle_list[turn_label_idle].size=t;
							   turn_idle_list[turn_label_idle].group=2;
						     } 
							delay_ms(30200);
                                           break;//·ÀÖ¹±¾Ñ­»·ÖÐ¶Ôi£¬ÖØ¸´Æ¥ÅäÍ¶ÇÐ

					   }
				    }
      	}
      	}
   }
}

void unload_power(status_list_node *list_1,status_list_node *list_2)
{
 u8 i,j;
 u8 label_busy1,label_busy2;
  turn_flag=1;
  led_on_off(BUSY_NODE_LCD_LCOK,0);
for(i=1;i<=slave[0];i++)inquiry_slave_status(slave[i]);
 label_busy2=sort_busynode_list_asc(sort_busy_list_2,list_2);
 label_busy1=sort_busynode_list_asc(sort_busy_list_1,list_1);
				


 
/***********************************************************************/
//label_busy1=0;//ÊµÑé Ö»Í¶ÇÐµÚ¶þ×éÓÃ £¬²úÆ·±ØÐëÈ¥µôÕâ¾ä»°
/************************************************************************/

	for(j=1;j<=label_busy1&&label_busy1>0;)
		{
                     delay_time(1);
			myled();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
		   if(gonglvshishu<95)break;
		   else {	
						if((wugong_computer-wugong_95)>sort_busy_list_1[label_busy1].size)
				   {
                             led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_1[label_busy1].myid);                                         											
				   order_trans_rs485(mybox.myid,sort_busy_list_1[label_busy1].myid,Sub_Order,1,0);
				   delay_ms(3500);
				   label_busy1--;
                              

				}

						if((wugong_computer-wugong_95)<=sort_busy_list_1[j].size)
				   {
	                             led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_1[j].myid);                                         														   
				   order_trans_rs485(mybox.myid,sort_busy_list_1[j].myid,Sub_Order,1,0);
				  delay_ms(3500);
                              j++;
				}

		          }
	    }

if(gonglvshishu>95)
 {
	for(j=1;j<=label_busy2&&label_busy2>0;)
		{
                        delay_time(1);
	                 myled();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
		   if(gonglvshishu<95)break;
		   else{	                               
                                	if((wugong_computer-wugong_95)>sort_busy_list_2[label_busy2].size)
				   {
				          led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_2[label_busy2].myid);                                         											
				   order_trans_rs485(mybox.myid,sort_busy_list_2[label_busy2].myid,Sub_Order,2,0);
				  delay_ms(3500);
				   label_busy2--;
                                                         
				}

			   if((wugong_computer-wugong_95)<=sort_busy_list_2[j].size)
		   	       {
				          led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_2[j].myid);                                         											
				   order_trans_rs485(mybox.myid,sort_busy_list_2[j].myid,Sub_Order,2,0);
				   delay_ms(3500);
				   j++;
			   }
                         				
   

		           }
        }

}

}

void C_unload_power(status_list_node *list_1,status_list_node *list_2)
{
 u8 i,j;
 u8 label_busy1,label_busy2;
  turn_flag=1;
  led_on_off(BUSY_NODE_LCD_LCOK,0);
for(i=1;i<=slave[0];i++)inquiry_slave_status(slave[i]);
 label_busy2=sort_busynode_list_asc(sort_busy_list_2,list_2);
 label_busy1=sort_busynode_list_asc(sort_busy_list_1,list_1);

	for(j=1;j<=label_busy1&&label_busy1!=0;)
		{

                     delay_time(1);
			myled();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
		   if(L_C_flag==1)break;
		   else {	
                            				if((wugong_computer)>sort_busy_list_1[label_busy1].size)
				                            { 									
								  led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_1[label_busy1].myid);
								order_trans_rs485(mybox.myid,sort_busy_list_1[label_busy1].myid,Sub_Order,1,0);
				                              delay_ms(3500);
				                                label_busy1--;
				                             }

                                                            if((wugong_computer)<=sort_busy_list_1[j].size)
			                                     { 
			                                     led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_1[j].myid);
                          			                  order_trans_rs485(mybox.myid,sort_busy_list_1[j].myid,Sub_Order,1,0);
					                          delay_ms(3500);
											  j++;
	                                                    }
		          }
	    }

if(L_C_flag==0)
 {
	for(j=1;j<=label_busy2&&label_busy2!=0;)
		{
                            delay_time(1);
				myled();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
		   if(L_C_flag==1)break;
		   else{	
                                                            if((wugong_computer)>sort_busy_list_2[label_busy2].size)
				   {
			                                     led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_2[label_busy2].myid);                                                          								
				   order_trans_rs485(mybox.myid,sort_busy_list_2[label_busy2].myid,Sub_Order,2,0);
				  delay_ms(3500);
				   label_busy2--;

				}

			 if((wugong_computer)<=sort_busy_list_2[j].size)
			{
                     led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_2[j].myid);                                      														   
			order_trans_rs485(mybox.myid,sort_busy_list_2[j].myid,Sub_Order,2,0);
			delay_ms(3500);	
			j++;

                     }

		   }
        }


}
}

u8 sort_idlenode_list(idle_list *sort_idle_list,status_list_node *list)//¿ÕÏÐÓÐÐò¶ÓÁÐ(°´ÈÝÁ¿´óÐ¡ÓÉ´óµ½Ð¡ÅÅÁÐ£¬·µ»Ø¿ÕÏÐ½Úµã¸öÊý)
{
   u8 i,j=1,k,t,flag=0;
   u8 count=0;
   for(i=1;i<=slave[0];i++){sort_idle_list[i].myid=0;sort_idle_list[i].size=0;}
   for(i=1;i<=slave[0];i++){
   	              if(list[slave[i]].work_status==0&&list[slave[i]].size!=0)
                      { sort_idle_list[j].myid=list[slave[i]].myid;
				        sort_idle_list[j].size=list[slave[i]].size;
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
for(i=1;i<count;i++)
   {if(sort_idle_list[i].myid==mybox.myid)
           {   t=sort_idle_list[i].size;
	        k=sort_idle_list[i].myid;
             for(j=i;j<count;j++)
              	{sort_idle_list[j].size=sort_idle_list[j+1].size;
                       sort_idle_list[j].myid=sort_idle_list[j+1].myid;
			 }
			 sort_idle_list[count].size=t;
			 sort_idle_list[count].myid=k;
			 break;
           }
    }
   	}
sort_idle_list[0].size=count;
	return count;
}


//Î´Íê³É
u8 offset_idlenode_one_list(offset_node *offset_idle_list,idle_list *sort_idle_list_1,idle_list *sort_idle_list_2)//¿ÕÏÐÓÐÐò¶ÓÁÐ(°´ÈÝÁ¿´óÐ¡ÓÉ´óµ½Ð¡ÅÅÁÐ£¬·µ»Ø¿ÕÏÐ½Úµã¸öÊý)
{
u8 i=1,j=1,k,len;
if(sort_idle_list_1[0].size==0&&sort_idle_list_2[0].size==0){offset_idle_list[0].size=0;return 0;}

else
{
for(i=1;i<=sort_idle_list_1[0].size;i++)
{offset_idle_list[i].myid=sort_idle_list_1[i].myid;
offset_idle_list[i].size=sort_idle_list_1[i].size;
offset_idle_list[i].group=1;
}
len=sort_idle_list_1[0].size;

for(i=1;i<=sort_idle_list_2[0].size;i++)
{
for(;j<=len;j++)
{
if(sort_idle_list_2[i].size>offset_idle_list[j].size)
   {
           for(k=len;k>=j;k--)
           	{
                 offset_idle_list[k+1].size=offset_idle_list[k].size;
	          offset_idle_list[k+1].myid=offset_idle_list[k].myid;
		   offset_idle_list[k+1].group=offset_idle_list[k].group;
					 
	       }
		   len++;
	offset_idle_list[j].size=sort_idle_list_2[i].size;
	offset_idle_list[j].myid=sort_idle_list_2[i].myid;
	offset_idle_list[j].group=2;
       break;
    }
if(j>len)
   { for(;i<=sort_idle_list_2[0].size;i++)
             {offset_idle_list[j].myid=sort_idle_list_1[i].myid;
               offset_idle_list[j].size=sort_idle_list_1[i].size;
               offset_idle_list[j].group=2;
                     j++;
             }

     }
}
}
offset_idle_list[0].size=sort_idle_list_1[0].size+sort_idle_list_2[0].size;

return offset_idle_list[0].size;

}



}






/**************½«µÚÒ»×é¿ÕÏÐ¶ÓÁÐºÍµÚ¶þ×é¿ÕÏÐ¶ÓÁÐ×é³ÉÒ»×é½øÐÐÅÅÐòÂÖ ÐÝ***************************/


u8 turn_idlenode_list(turn_node *turn_idle_list,status_list_node *list_1,status_list_node *list_2)//¿ÕÏÐÓÐÐò¶ÓÁÐ(°´ÈÝÁ¿´óÐ¡ÓÉ´óµ½Ð¡ÅÅÁÐ£¬·µ»Ø¿ÕÏÐ½Úµã¸öÊý)
{
   u8 i,j=1,k,t,g,flag=0;
   u8 count=0;
   for(i=1;i<=(2*slave[0]);i++){turn_idle_list[i].myid=0;turn_idle_list[i].size=0;turn_idle_list[i].group=0;}
   for(i=1;i<=slave[0];i++){
   	              if(list_1[slave[i]].work_status==0&&list_1[slave[i]].size!=0)
                      { turn_idle_list[j].myid=list_1[slave[i]].myid;
				        turn_idle_list[j].size=list_1[slave[i]].size;
						turn_idle_list[j].group=1;
					          j++;
						count++;
						if(flag==0)flag=1;//Èç¹ûÃ»ÓÐ¿ÕÏÐ½Úµã
   	              	  }
                    }

for(i=1;i<=slave[0];i++)
		{
   	              if(list_2[slave[i]].work_status==0&&list_2[slave[i]].size!=0)
                      { turn_idle_list[j].myid=list_2[slave[i]].myid;
				        turn_idle_list[j].size=list_2[slave[i]].size;
						turn_idle_list[j].group=2;
					          j++;
						count++;
						if(flag==0)flag=1;//Èç¹ûÃ»ÓÐ¿ÕÏÐ½Úµã
   	              	  }
                    }
   
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=turn_idle_list[i].size;
	   k=turn_idle_list[i].myid;
	   g=turn_idle_list[i].group;
	   for(j=i-1;j>=1&&t>turn_idle_list[j].size;j--)
	   	{turn_idle_list[j+1].myid=turn_idle_list[j].myid;
                turn_idle_list[j+1].size=turn_idle_list[j].size;
		  turn_idle_list[j+1].group=turn_idle_list[j].group;
	    }
	            turn_idle_list[j+1].myid=k;
	           turn_idle_list[j+1].size=t;
			turn_idle_list[j+1].group=g;

      }
for(i=1;i<count;i++)
   {if(turn_idle_list[i].myid==mybox.myid)
           {   t=turn_idle_list[i].size;
	        k=turn_idle_list[i].myid;
		g=turn_idle_list[i].group;
             for(j=i;j<count;j++)
              	{turn_idle_list[j].size=turn_idle_list[j+1].size;
                       turn_idle_list[j].myid=turn_idle_list[j+1].myid;
			turn_idle_list[j].group=turn_idle_list[j+1].group;
			 }
			 turn_idle_list[count].size=t;
			 turn_idle_list[count].myid=k;
			 turn_idle_list[count].group=g;
			 break;
           }
    }
   	}
   //turn_idle_list[0]=count;
    return count;
}








/***************************************************************************************************************************/







/*
void sort_timenode_list(time_list *sort_time_list,status_list_node *list)//Ê±¼äÓÐÐò¶ÓÁÐ(°´¹¤×÷Ê±¼äÓÉ´óµ½Ð¡ÅÅÁÐ)
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
u8 sort_busynode_list(busy_list *sort_busy_list,status_list_node *list)//Ã¦ÂµÓÐÐò¶ÓÁÐ(°´¹¤×÷Ê±¼ä´óÐ¡ÓÉ´óµ½Ð¡ÅÅÁÐ)
{
   u8 i,j=1,g,f,k,t,w,flag=0;
   u8 count=0,count_time=0;
   for(i=1;i<=slave[0];i++){sort_busy_list[i].myid=0;sort_busy_list[i].size=0;}
   for(i=1;i<=slave[0];i++){
   	              if(list[slave[i]].work_status==1&&list[slave[i]].size!=0)
                      { sort_busy_list[j].myid=list[slave[i]].myid;
				        sort_busy_list[j].size=list[slave[i]].size;
						sort_busy_list[j].work_time=list[slave[i]].work_time;
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
	   for(j=i-1;j>=1&&t>sort_busy_list[j].size;j--)
	   	{sort_busy_list[j+1].myid=sort_busy_list[j].myid;
         sort_busy_list[j+1].size=sort_busy_list[j].size;
		 sort_busy_list[j+1].work_time=sort_busy_list[j].work_time;
	    }
	   sort_busy_list[j+1].myid=k;
	   sort_busy_list[j+1].size=t;
       sort_busy_list[j+1].work_time=w;
     }
   /**********************************************/
   for(i=1;i<=count;i=i+count_time)
      { count_time=0;
        for(j=i;j<=count;j++)
        	{ if(sort_busy_list[i].size==sort_busy_list[j].size){count_time++;}
                     else break;
		}
		
		for(g=i+1;g<=i+count_time-1;g++)
			{
                               t=sort_busy_list[g].size;
	                        k=sort_busy_list[g].myid;
	                        w=sort_busy_list[g].work_time;
                 	   for(f=g-1;f>=i&&w>sort_busy_list[f].work_time;f--)
	   	{
	   	 sort_busy_list[f+1].myid=sort_busy_list[f].myid;
               sort_busy_list[f+1].size=sort_busy_list[f].size;
		 sort_busy_list[f+1].work_time=sort_busy_list[f].work_time;
	       }
	      sort_busy_list[f+1].myid=k;
	      sort_busy_list[f+1].size=t;
             sort_busy_list[f+1].work_time=w;
		       }

      }
           
   
   	}
   //sort_busy_list[0]=count;
  return count;
}


u8 sort_busynode_list_asc(busy_list *sort_busy_list,status_list_node *list)//Ã¦ÂµÓÐÐò¶ÓÁÐ(°´¹¤×÷ÈÝÁ¿´óÐ¡ÓÉ´óµ½Ð¡ÅÅÁÐ)
{
   u8 i,j=1,g,f,k,t,w,flag=0;
   u8 count=0,count_time=0;
   for(i=1;i<=slave[0];i++){sort_busy_list[i].myid=0;sort_busy_list[i].size=0;}
   for(i=1;i<=slave[0];i++){
   	              if(list[slave[i]].work_status==1&&list[slave[i]].size!=0)
                      { sort_busy_list[j].myid=list[slave[i]].myid;
				        sort_busy_list[j].size=list[slave[i]].size;
						sort_busy_list[j].work_time=list[slave[i]].work_time;
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
   /**********************************************/
   for(i=1;i<=count;i=i+count_time)
      { count_time=0;
        for(j=i;j<=count;j++)
        	{ if(sort_busy_list[i].size==sort_busy_list[j].size){count_time++;}
                     else break;
		}
		
		for(g=i+1;g<=i+count_time-1;g++)
			{
                               t=sort_busy_list[g].size;
	                        k=sort_busy_list[g].myid;
	                        w=sort_busy_list[g].work_time;
                 	   for(f=g-1;f>=i&&w>sort_busy_list[f].work_time;f--)
	   	{
	   	 sort_busy_list[f+1].myid=sort_busy_list[f].myid;
               sort_busy_list[f+1].size=sort_busy_list[f].size;
		 sort_busy_list[f+1].work_time=sort_busy_list[f].work_time;
	       }
	      sort_busy_list[f+1].myid=k;
	      sort_busy_list[f+1].size=t;
             sort_busy_list[f+1].work_time=w;
		       }

      }
           
   
   	}
     // sort_busy_list[0]=count;
  return count;
}







void delay_time(u32 time)
{ heartbeat(time);
}  //±¾ÏµÍ³µÄÑÓÊ±º¯Êý£¬time*10ms

u8 inquiry_slave_status(u8 id)   
  {  u8 *msg;
        u8 err;
			if(id==mybox.myid)
		{
	   set_statuslist_1(mystatus.myid,mystatus.size[0],mystatus.work_status[0],mystatus.work_time[0]);//Ö÷»ú×´Ì¬ÐÅÏ¢Ð´Èë×´Ì¬±í
       set_statuslist_2(mystatus.myid,mystatus.size[1],mystatus.work_status[1],mystatus.work_time[1]);
	   return 1;
		}

   order_trans_rs485(mybox.myid,id,2,0,0);
  // delay_us(10000);
   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/50,&err);
   if(err==OS_ERR_TIMEOUT){set_statuslist_1(id,0,2,0);set_statuslist_2(id,0,2,0);return 0;}//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
	{  rs485_trans_status(msg);return 1;}

} //²éÑ¯´Ó»ú×´Ì¬²¢±£´æµ½´Ó»ú×´Ì¬±íÖÐ£¬²ÎÊýidÊÇÒª²éÑ¯µÄ´Ó»úºÅ






/*******************¹¦ÂÊÒòËØÏà¹Øº¯Êý*****************************/


void gonglvyinshu()
{
        u16 i;
		u32 tempa,tempb;
		u16 adc_vx,adc_vmax=0,adc_ix,adc_imax=0;
		u8 phase_zhi;
		 float temp;

		id_num=AT24CXX_ReadOneByte(0x0010);
	

		 for(i=0;i<80;i++)
//	for(i=0;i<120;i++)
	  	 {
	  	 adc_vx=Get_Adc_Average(ADC_Channel_1,10);
		  // adc_vx=Get_Adc(ADC_Channel_1);
		   if(adc_vx>adc_vmax)
		   adc_vmax=adc_vx;
	  	 }
	   for(i=0;i<80;i++)
	 //for(i=0;i<120;i++)
	  {
		 adc_ix=Get_Adc_Average(ADC_CH4,10);
		 if(adc_ix>adc_imax)
		 adc_imax=adc_ix;
	  }

    	  

           
	   
	  temp=((float)adc_vmax*(3.3/4096))*1000;
	dianya_zhi=((u16)(5*temp-6205))/10;
	  temp=((float)adc_imax*(3.3/4096))*1000;
	 dianliuzhi=(((u32)(532*temp-656900))/1000)*(k/100);
        // dianliuzhi=temp;
	  adc_vmax=0;
	  adc_imax=0;
	  	 // if(dianliuzhi<=1){dianliuzhi=0;gonglvshishu=100;}//ÂË³ýÔÓ²¨£¬Ð¡ÓÚ7Ê±£¬ËµÃ÷ÒÑ¾­ÎÞ¸ºÔØ

		 
	  if(TIM3CH1_CAPTURE_STA&0X80)//Íê³ÉÒ»´Î²É¼¯
		{
			tempa=TIM3CH1_CAPTURE_STA&0X3F;
			tempa*=65536;					//Òç³öÊ±¼ä×ÜºÍ
			tempa+=TIM3CH1_CAPTURE_VAL;		//µÃµ½TI1¶ËÐÅºÅÖÜÆÚÊ±¼ä
                                tempa=20000;                             //ÖÜÆÚÐ´ËÀ£¬²»ÔÙ¼ÆËã

			tempb=TIM3CH1_CAPTURE_STA&0X3F;
			tempb*=65536;					//Òç³öÊ±¼ä×ÜºÍ
			tempb+=TIM3CH1_CAPTURE_PHA;		//µÃµ½TI2 TI1ÉÏÉýÑØÊ±¼ä²îÖµ¼´ÏàÎ»²îÊ±¼ä

	 		 if(tempb<=5000)			   //¸ÐÐÔ¸ºÔØÕý½Ó
			 {
			 	phase_zhi=(tempb*360/tempa);
		   	 	gonglvshishu=(si[phase_zhi]);
				
				/********************¹¦ÂÊÒòËØ²»¾«È·ÊÊµ±µ÷Õû************************************
				if(gonglvshishu>69&&gonglvshishu<=89)gonglvshishu=gonglvshishu+2;
				if(gonglvshishu>=90&&gonglvshishu<94&&gonglvshishu!=90&&gonglvshishu!=91)gonglvshishu=gonglvshishu+2;
				if(gonglvshishu>=94&&(gonglvshishu!=96&&gonglvshishu!=98&&gonglvshishu!=99&&gonglvshishu!=100))gonglvshishu=gonglvshishu+1;
				***********************************************************/
				L_C_flag=1;
			 }
			  if((10000<=tempb)&&(tempb<=15000))			 //¸ÐÐÔ¸ºÔØ·´½Ó
			 {
			 	phase_zhi=(((tempb*360)/tempa)-180);
		   	 	gonglvshishu=(si[phase_zhi]);
				/********************¹¦ÂÊÒòËØ²»¾«È·ÊÊµ±µ÷Õû***********************************
				if(gonglvshishu>69&&gonglvshishu<=89)gonglvshishu=gonglvshishu+2;
				if(gonglvshishu>=90&&gonglvshishu<94&&gonglvshishu!=90&&gonglvshishu!=91)gonglvshishu=gonglvshishu+2;
				if(gonglvshishu>=94&&(gonglvshishu!=96&&gonglvshishu!=98&&gonglvshishu!=99&&gonglvshishu!=100))gonglvshishu=gonglvshishu+1;
				************************************************************/
				L_C_flag=1;
			 }
			 if((5000<tempb)&&(tempb<10000))	   //ÈÝÐÔ¸ºÔØÕý½Ó
			 {
				/*ÏÔÊ¾ÈÝÐÔ¹¦ÂÊ·ûºÅ*/
			 	phase_zhi=(180-tempb*360/tempa);
		   	 	gonglvshishu=(si[phase_zhi]);
				/********************¹¦ÂÊÒòËØ²»¾«È·ÊÊµ±µ÷Õû***********************************
				if(gonglvshishu>69&&gonglvshishu<=89)gonglvshishu=gonglvshishu-2;
				if(gonglvshishu>=90&&(gonglvshishu!=99&&gonglvshishu!=100))gonglvshishu=gonglvshishu-2;
				***********************************************************/

				L_C_flag=0;
			 }
			 if((15000<tempb)&&(tempb<20000))	   //ÈÝÐÔ¸ºÔØ·´½Ó
			 {
				/*ÏÔÊ¾ÈÝÐÔ¹¦ÂÊ·ûºÅ*/
			 	phase_zhi=(360-tempb*360/tempa);
		   	 	gonglvshishu=(si[phase_zhi]);
								/********************¹¦ÂÊÒòËØ²»¾«È·ÊÊµ±µ÷Õû***********************************
				if(gonglvshishu>69&&gonglvshishu<=89)gonglvshishu=gonglvshishu-2;
				if(gonglvshishu>=90&&(gonglvshishu!=99&&gonglvshishu!=100))gonglvshishu=gonglvshishu-2;
				*********************************************************/

				L_C_flag=0;
			 }

	  	
			 wugongkvar=(uint16_t)((1.732*dianliuzhi*dianya_zhi*(co[phase_zhi]))/100000);
			wugong_95= (uint16_t)((17.32*dianliuzhi*dianya_zhi*31)/100000);//¹¦ÂÊÒòËØÔÚ0.95Ê±µÄ£¬ÎÞ¹¦¹¦Â
			wugong_computer=(uint16_t)((17.32*dianliuzhi*dianya_zhi*co[phase_zhi])/100000);
                    //wugongkvar=wugong_computer;
			TIM3CH1_CAPTURE_STA=0;			//¿ªÆôÏÂÒ»´Î²¶»ñ
			
		
		}

//ÎÞ¹¦¹¦ÂÊ
}

void temperature()   //µçÈÝÆ÷ÎÂ¶È¼ì²â
{
 u16 adc_tmp1=0;
 //u16 adc_tmp2=0;
       adc_tmp1=Get_Adc_Average(ADC_Channel_5,10);
//	  adc_tmp2=Get_Adc_Average(ADC_Channel_6,10);
	  tempshuzhi=(u8)(258-((adc_tmp1*255)/4096));

}

void LIGHT(u8 status_1,u8 status_2)
{
if(status_1==0&&status_2==1)HT595_Send_Byte((GREEN_RED)|background_light_on);
if(status_1==1&&status_2==0)HT595_Send_Byte((RED_GREEN)|background_light_on);
if(status_1==0&&status_2==0)HT595_Send_Byte((GREEN_GREEN)|background_light_on);
if(status_1==1&&status_2==1)HT595_Send_Byte((RED_RED)|background_light_on);
if(status_1==2&&status_2==2)HT595_Send_Byte((YELLOW_YELLOW)|background_light_on);

}

void myled()
  {
gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
temperature();
key_lcd();
delay_ms(50);//Ã»ÓÐÑÓÊ±£¬ÆÁ»áËÀ»ú
}

void Alarm(void)
{
	   if((tempshuzhi>=70||dianya_zhi>=440||dianya_zhi<=340)&&alarm_lock==0)
	   	{   
                                        if(mybox.master==1)//Ö÷»ú·¢²¼Í¨ÖªÑÓÊ±ÐÅÏ¢
								  {
								       delay_time(80);
								  }   
				GPIO_SetBits(GPIOA,GPIO_Pin_0);
					delay_us(100000);
                    GPIO_SetBits(GPIOA,GPIO_Pin_8);
		       set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],2,2,0,0);
		       LIGHT(mystatus.work_status[0],mystatus.work_status[1]);//¹ýÎÂ³£Ì¬
		       alarm_lock=1;

	   }
	   
	/*	   if((dianya_zhi>=440||dianya_zhi<=340)&&mybox.master==1)//Èç¹ûÊÇ¹ýÑ¹»òÇ·Ñ¹²¢ÇÒ Èç¹ûÊÇÖ÷»ú½øÈë¸ÃÑ­»·
		   	{
                             while(1)
					{   gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
                                        temperature();
                                           key_idset();//°´¼üÓëÏÔÊ¾¹¦ÄÜ
                                             delay_time(1);
						delay_ms(100);//Ã»ÓÐÑÓÊ±£¬ÆÁ»áËÀ»ú

					if(dianya_zhi<440&&dianya_zhi>340)break;						 

				        }

		      }
		      
ËÀ»úÎðÓÃ
*/
		  if(tempshuzhi<70&&dianya_zhi<440&&dianya_zhi>340&&alarm_lock==1)
	         {

			   if(mybox.master==1)//Ö÷»ú·¢²¼Í¨ÖªÑÓÊ±ÐÅÏ¢
								  {
								       delay_time(80);
								  }
		    set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,0,0,0);
		    LIGHT(mystatus.work_status[0],mystatus.work_status[1]);//»Ö¸´³£Ì¬
		    alarm_lock=0;
	 	}

	
	 

}
	 
void key_lcd()
{
key_idset();//°´¼üÓëÏÔÊ¾¹¦ÄÜ
Alarm();//ÊÇ·ñÐèÒª±¨¾¯
 LIGHT(mystatus.work_status[0],mystatus.work_status[1]);//Ë¢Ö¸Ê¾µÆ£¬Èç¹ûÏÔÊ¾Æ÷ÓÐÅÔÂ·µçÈÝÂË²¨ ¿ÉÒÔÉ¾³ý
}


void scanf_slave_machine()
{
u8 i,j;
u8 count=1;
for(i=1;i<33;i++)
	{  
	j=inquiry_slave_status(i);
        if(j==1){slave[count]=i;count++;}
       }
      slave[0]=count-1;
	  
}

u8 sub_delaytime_15(u8 i)
{
if(i==0)
{return 0;}

if(i<=32)
{            
                     if((i)<=4)
               { delay_ms(i*15000);}
			 if((i)>4&&(i)<=8)
			                     {  delay_ms(4*15000);
			 	                delay_ms(((i-4)*15000));
			 	               }		
               			 if(i>8&&i<=12)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
			 	                delay_ms(((i-8)*15000));
			 	               }		
			                 if(i>12&&i<=16)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
							delay_ms(4*15000);
			 	                delay_ms(((i-12)*15000));
			 	               }		
                                   if(i>16&&i<=20)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
			 	                delay_ms(((i-16)*15000));
      			 	               }		
                                      if(i>20&&i<=24)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						     delay_ms(4*15000);
			 	                delay_ms(((i-20)*15000));
			 	               }		
 
                                      if(i>24&&i<=28)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						     delay_ms(4*15000);
							 delay_ms(4*15000);
			 	                delay_ms(((i-24)*15000));
			 	               }		
 
                                      if(i>28&&i<=32)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						     delay_ms(4*15000);
							 delay_ms(4*15000);
							 delay_ms(4*15000);
			 	                delay_ms(((i-28)*15000));
			 	               }		                             


}
if(i>32)
{
delay_ms(4*15000);
delay_ms(4*15000);
delay_ms(4*15000);
delay_ms(4*15000);
delay_ms(4*15000);
delay_ms(4*15000);
delay_ms(4*15000);
delay_ms(4*15000);
i=i-32;
                     if((i)<=4)
               { delay_ms(i*15000);}
			 if((i)>4&&(i)<=8)
			                     {  delay_ms(4*15000);
			 	                delay_ms(((i-4)*15000));
			 	               }		
               			 if(i>8&&i<=12)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
			 	                delay_ms(((i-8)*15000));
			 	               }		
			                 if(i>12&&i<=16)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
							delay_ms(4*15000);
			 	                delay_ms(((i-12)*15000));
			 	               }		
                                   if(i>16&&i<=20)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
			 	                delay_ms(((i-16)*15000));
      			 	               }		
                                      if(i>20&&i<=24)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						     delay_ms(4*15000);
			 	                delay_ms(((i-20)*15000));
			 	               }		
 
                                      if(i>24&&i<=28)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						     delay_ms(4*15000);
							 delay_ms(4*15000);
			 	                delay_ms(((i-24)*15000));
			 	               }		
 
                                      if(i>28&&i<=32)
			                     {  delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						    delay_ms(4*15000);
						     delay_ms(4*15000);
							 delay_ms(4*15000);
							 delay_ms(4*15000);
			 	                delay_ms(((i-28)*15000));
			 	               }		                             


}

return 0;
}

u8 sub_delaytime_5(u8 i)
{
if(i==0)
{return 0;}


if(i<=32)
{
                     if((i)<=4)
               { delay_ms(i*5000);}
			 if((i)>4&&(i)<=8)
			                     {  delay_ms(4*5000);
			 	                delay_ms(((i-4)*5000));
			 	               }		
               			 if(i>8&&i<=12)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
			 	                delay_ms(((i-8)*5000));
			 	               }		
			                 if(i>12&&i<=16)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
							delay_ms(4*5000);
			 	                delay_ms(((i-12)*5000));
			 	               }		
                                   if(i>16&&i<=20)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
			 	                delay_ms(((i-16)*5000));
      			 	               }		
                                      if(i>20&&i<=24)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						     delay_ms(4*5000);
			 	                delay_ms(((i-20)*5000));
			 	               }		
 
                                      if(i>24&&i<=28)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						     delay_ms(4*5000);
							 delay_ms(4*5000);
			 	                delay_ms(((i-24)*5000));
			 	               }		
 
                                      if(i>28&&i<=32)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						     delay_ms(4*5000);
							 delay_ms(4*5000);
							 delay_ms(4*5000);
			 	                delay_ms(((i-28)*5000));
			 	               }		                             


}
if(i>32)
{
delay_ms(4*5000);
delay_ms(4*5000);
delay_ms(4*5000);
delay_ms(4*5000);
delay_ms(4*5000);
delay_ms(4*5000);
delay_ms(4*5000);
delay_ms(4*5000);
i=i-32;
                     if((i)<=4)
               { delay_ms(i*5000);}
			 if((i)>4&&(i)<=8)
			                     {  delay_ms(4*5000);
			 	                delay_ms(((i-4)*5000));
			 	               }		
               			 if(i>8&&i<=12)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
			 	                delay_ms(((i-8)*5000));
			 	               }		
			                 if(i>12&&i<=16)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
							delay_ms(4*5000);
			 	                delay_ms(((i-12)*5000));
			 	               }		
                                   if(i>16&&i<=20)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
			 	                delay_ms(((i-16)*5000));
      			 	               }		
                                      if(i>20&&i<=24)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						     delay_ms(4*5000);
			 	                delay_ms(((i-20)*5000));
			 	               }		
 
                                      if(i>24&&i<=28)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						     delay_ms(4*5000);
							 delay_ms(4*5000);
			 	                delay_ms(((i-24)*5000));
			 	               }		
 
                                      if(i>28&&i<=32)
			                     {  delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						    delay_ms(4*5000);
						     delay_ms(4*5000);
							delay_ms(4*5000);
							delay_ms(4*5000);
			 	                delay_ms(((i-28)*5000));
			 	               }		                             


}

	return 0;
}



