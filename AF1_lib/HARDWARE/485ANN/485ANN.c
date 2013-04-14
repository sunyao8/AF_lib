#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "485ANN.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "key.h"
/***********************************************************************/
u16 RS485_RX_BUF[64]; 		//���ջ���,���64���ֽ�
//���յ������ݳ���
u8 RS485_RX_CNT=0;  
//ģʽ����
 u16  dog_clock=10;

 OS_EVENT * RS485_MBOX,* RS485_STUTAS_MBOX;			//	rs485�����ź���
 OS_EVENT *Heartbeat;			 //�����ź���

u8 cont=0;//���ڸ��������ŵļǴ�����
u32 life_time_1=0;
u32 life_time_2=0;
u8 flag=1;

box mybox;
status_box mystatus;
idle_list sort_idle_list_1[33];
idle_list sort_idle_list_2[33];
busy_list sort_busy_list_1[33];
busy_list sort_busy_list_2[33];

status_list_node system_status_list_1[33];
status_list_node system_status_list_2[33];

//u8 rs485buf[LEN_control];//���Ϳ�����Ϣ
u8 rs485buf[LEN_control];//���Ϳ�����Ϣ
u8 statusbuf[LEN_status];//����״̬��Ϣ
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

extern u8 id_num; 
extern u8 grafnum,tempshuzhi,gonglvshishu;
extern u16 dianya_zhi,	wugongkvar,k;
extern u32	dianliuzhi;
/*****************************************************/
 void TIM4_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM4��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM4�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIMx					 
}
 
 void TIM4_IRQHandler(void)   //TIM4�ж�
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //���TIM4�����жϷ������
		{	  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIMx�����жϱ�־
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
		 if (mystatus.work_status[0]==1)  //����ʱ��ļ�ʱ
		    { // life_time_1++;
		       //if(life_time_1==4)
			   { mystatus.work_time[0]++;
			    // life_time_1=0;
				 if(mystatus.work_time[0]==37)mystatus.work_time[0]=39;
				  if(mystatus.work_time[0]==254)mystatus.work_time[0]=0;
			   }
		 	}
		 if(mystatus.work_status[1]==1)
		 	{ //life_time_2++;
              //if(life_time_2==4)
			  	{  mystatus.work_time[1]++;
			      // life_time_2=0;
                   if(mystatus.work_time[1]==37)mystatus.work_time[1]=39;
			 if(mystatus.work_time[1]==254)mystatus.work_time[1]=0;	   
			    }
		     }
		}
   	OSIntExit();  
}
//////////////////////////////////////////////////////////////////////////////////
void TIM3_Cap_Init(u16 arr,u16 psc)
{	NVIC_InitTypeDef NVIC_InitStructure;	 
	RCC->APB1ENR|=1<<1;   	//TIM3 ʱ��ʹ�� 
	RCC->APB2ENR|=1<<3;    	//ʹ��PORTBʱ��  
	 
	GPIOB->CRL&=0XFFFFFF00;	//PB0 PB1���֮ǰ����  
	GPIOB->CRL|=0X00000088;	//PB0 PB1����   
	GPIOB->ODR&=~(1<<0);		//PB0 PB1����
	GPIOB->ODR&=~(1<<1);
	  
 	TIM3->ARR=arr;  		//�趨�������Զ���װֵ   
	TIM3->PSC=psc;  		//Ԥ��Ƶ�� 

	TIM3->CCMR2|=1<<0;		//CC3S=01 	ѡ������� IC3ӳ�䵽TI3��
 	TIM3->CCMR2|=0<<4; 		//IC3F=0000 ���������˲��� ���˲�
 	TIM3->CCMR2|=0<<2;  	//IC3PS=00 	���������Ƶ,����Ƶ 

	TIM3->CCER|=0<<9; 		//CC3P=0	�����ز���1
	TIM3->CCER|=1<<8; 		//CC3E=1 	�����������1��ֵ������Ĵ�����

	TIM3->DIER|=1<<3;   	//������3�ж�				
	TIM3->DIER|=1<<0;   	//�������1�ж�	

	TIM3->CCMR2|=1<<8;		//CC4S=01 	ѡ������� IC4ӳ�䵽TI4��
 	TIM3->CCMR2|=0<<12; 	//IC4F=0000 ���������˲��� ���˲�
 	TIM3->CCMR2|=0<<10; 	//IC4PS=00 	���������Ƶ,����Ƶ 

	TIM3->CCER|=0<<13; 		//CC4P=0	�����ز���
	TIM3->CCER|=1<<12; 		//CC4E=1 	�������������ֵ������Ĵ�����

	TIM3->DIER|=1<<4;   	//������4�ж�				
	TIM3->DIER|=1<<0;   	//��������ж�
	TIM3->CR1|=0x01;    	//ʹ�ܶ�ʱ��2
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  //�����ȼ�4��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
	   
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽸ߵ�ƽ;1,�Ѿ����񵽸ߵ�ƽ��.
//[5:0]:����ߵ�ƽ������Ĵ���
u8  TIM3CH1_CAPTURE_STA=0;	//���벶��״̬		    				
u16	TIM3CH1_CAPTURE_VAL;	//���벶��ֵ
u16	TIM3CH1_CAPTURE_PHA;	//���벶��ֵ
//��ʱ��3�жϷ������	 
void TIM3_IRQHandler(void)
{ 		    
	u16 tsr;
	tsr=TIM3->SR;
 	if((TIM3CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if(tsr&0X01)//���
		{	    
			if(TIM3CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM3CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM3CH1_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					TIM3CH1_CAPTURE_VAL=0XFFFF;
				}else TIM3CH1_CAPTURE_STA++;
			}	 
		}
		if(tsr&0x10)//����1���������¼�
		{	
			if(TIM3CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				TIM3CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			    TIM3CH1_CAPTURE_VAL=TIM3->CCR4;	//��ȡ��ǰ�Ĳ���ֵ.
	 			TIM3->CCER&=~(1<<1);			//CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM3CH1_CAPTURE_STA=0;			//���
				TIM3CH1_CAPTURE_VAL=0;
				TIM3CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
	 			TIM3->CNT=0;					//���������
			  	TIM3->CCER&=~(1<<1);			//CC1P=0 ����Ϊ�����ز���
			}		    
		}
		if(tsr&0x08)
		{
		 	if(TIM3CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
			    TIM3CH1_CAPTURE_PHA=TIM3->CCR3;	//��ȡ��ǰ�Ĳ���ֵ.
			}
		}			     	    					   
 	}
	TIM3->SR=0;//����жϱ�־λ 	    
}


//////////////////////////////////////////////////////////
		void USART2_IRQHandler(void)
{
 
	u8 RS485_RX_BUF[64];
	#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntEnter();    
      #endif
 		
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //���յ�����
	{	 
	 			 
		 RS485_RX_BUF[RS485_RX_CNT++]=USART_ReceiveData(USART2); 	//��ȡ���յ�������
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='&'){RS485_RX_BUF[0]='&'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='*')
		{
				RS485_RX_CNT=0;

				
				if(RS485_RX_BUF[1]=='#'){OSMboxPost(RS485_STUTAS_MBOX,(void*)&RS485_RX_BUF);}
				  else OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);
		} 
	}  	
	#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntExit();  											 
#endif

} 

void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			//����Ϊ����ģʽ
  	for(t=0;t<len;t++)		//ѭ����������
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);		
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;				//����Ϊ����ģʽ	

}

void initmybox(u8 id)//��ʼ��������Ϣ
{  	 
  
 // for(i=1;i<33;i++)token[i]=0;//��ʼ������
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

void turn_master_id(u8 id)//�ı䵱ǰ����ϵͳ��������ID��
{
   u8 i,flag=0;
	{ 
	  flag=cont;
      if(id==(flag)){
	  	for(i=1;i<33;i++)
			{ order_trans_rs485(mybox.myid,i,0,0,0);
		     delay_us(10000);
			}//��ʱ��֪����slave��������������
         mybox.master=1;
	    OSTaskResume(MASTER_TASK_PRIO);
		cont=1;
	  }
	 //  LED1=!LED1;
	  
      }
   }






 int rs485_trans_order(u8 *tx_r485)//�������������͹������źţ������͸���λ��
{ 
  dianya_zhi=comp_16(tx_r485[6],tx_r485[7]);
  dianliuzhi=comp_16(tx_r485[8],tx_r485[9]);
  wugongkvar=comp_16(tx_r485[10],tx_r485[11]);
  tempshuzhi=tx_r485[12];
  gonglvshishu=tx_r485[13];
   if(mybox.myid==tx_r485[2]||tx_r485[2]==0)//�ж��Ƿ��Ƿ�����������Ϣ���ǹ㲥��Ϣ
   	{
   	 mybox.source=tx_r485[1];
   	 mybox.send=tx_r485[3];
     mybox.relay=tx_r485[4];
     mybox.message=tx_r485[5];
     return 1;
   	}
   else return 0;
}

 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message)//���������������������RS485��Ϣ�����͸�Ŀ�Ĵӻ�
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
      rs485buf[0]='&';//Э��ͷ
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
	rs485buf[14]='*';//Э��β
	RS485_Send_Data(rs485buf,15);//����5���ֽ�
	if(destination==source)subcontrol(relay, message);//�����Ϣ�������Լ�
	OS_EXIT_CRITICAL();	
}

u16 comp_16(u16 a,u16 b)
{
u16 value=0;
value=((a&0x00FF)+((b<<8)&0xFF00));
return value;
}


void Heartbeat_task(void *pdata)//master����������
{		// u8 key;
        u8 i=0;
		u8 err;
	//  u8 rs485buf[5];
	while(1)
	{
	OSSemPend(Heartbeat,0,&err);
		for(i=1;i<33;i++)
		{	
	       order_trans_rs485(mybox.myid,i,0,0,0);
		    delay_us(10000);
		  // LCD_ShowxNum(60+i*32,190,0,7,16,0X80);
		}		 
				   
	}
}

void led_on_off(u8 on_off) //����ֵ1 Ϊ��led ��0Ϊ�ر�led
{
u8 i;
if(on_off==1)
    {
        for(i=1;i<33;i++)
	   { order_trans_rs485(mybox.myid,0,4,0,0);
             delay_us(10000);
          }	 
    }
if(on_off==0)
    {
        for(i=1;i<33;i++)
	   { order_trans_rs485(mybox.myid,0,3,0,0);
             delay_us(10000);
          }	 
    }
}


/*****************************������Ϣ����********************************************/


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



 void status_trans_rs485(status_box *mystatus)//�ӻ�����
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
	RS485_Send_Data(statusbuf,10);//����10���ֽ�
	OS_EXIT_CRITICAL();	
}


 void rs485_trans_status(u8 *tx_r485)//���������������������RS485��Ϣ�����͸�Ŀ�Ĵӻ�
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


void offset_idlepower(status_list_node *list_1,status_list_node *list_2)  //���ʲ����������������� �޹����� �������� ���ж���
{
 u8 i,j;
 s8 label_idle1,label_idle2;
 u16 glys;
  flag=1;
for(i=1;i<33;i++)inquiry_slave_status(i);
 label_idle1=sort_idlenode_list(sort_idle_list_1,list_1);
	for(j=label_idle1;j>0;j--)
		{
           delay_time(1);
           glys=gonglvyinshu();
		   if(glys>90)break;
		   	order_trans_rs485(mybox.myid,sort_busy_list_1[j].myid,1,1,1);
        }
 	if(glys<90)
 		{
for(i=1;i<33;i++)inquiry_slave_status(i);
 label_idle1=sort_idlenode_list(sort_idle_list_2,list_2);
	for(j=label_idle2;j>0;j--)
		{
           delay_time(1);
           glys=gonglvyinshu();
		   if(glys>90)break;
		   	order_trans_rs485(mybox.myid,sort_busy_list_2[j].myid,1,2,1);
		    
	    }
 		}
}

void turn_power(status_list_node *list_1,status_list_node *list_2)//���������������ȶ������󣬽�����������ݺ���
{
u8 i,j,k,t,q;
s8 label_idle1,label_idle2;
s8 label_busy1,label_busy2;
   if(flag==1)
   {
   for(i=1;i<33;i++)inquiry_slave_status(i); 
   label_idle1=sort_idlenode_list(sort_idle_list_1,list_1);//�õ����ж���
   label_idle2=sort_idlenode_list(sort_idle_list_2,list_2);
   }  
    flag=0;
 //glys=gonglvyinshu();
   //if(glys<92||glys>95)break;//���������92��95֮�������ѭ�������µ��б�����������µõ����ж���
   for(i=1;i<33;i++)inquiry_slave_status(i); 
    label_busy1=sort_busynode_list(sort_busy_list_1,list_1);//ˢ��list_1��busy���ÿ�������ڵ�Ĺ���ʱ��
    label_busy2=sort_busynode_list(sort_busy_list_2,list_2);//ˢ��list_2��busy���ÿ�������ڵ�Ĺ���ʱ��
	if(label_idle1!=0)
     { for(i=1;i<=label_busy1;i++)
	  	{if(sort_busy_list_1[i].work_time>=TIME_OUT)
      	      {  
                for(j=1;j<=label_idle1;j++)
                	{  
                       if(sort_busy_list_1[i].size==sort_idle_list_1[j].size)
                       	{ order_trans_rs485(mybox.myid,sort_idle_list_1[j].myid,1,1,1);
                          order_trans_rs485(mybox.myid,sort_busy_list_1[i].myid,1,1,0);
						  	{ k=sort_busy_list_1[i].myid;
                              t=sort_busy_list_1[i].size;
                              sort_busy_list_1[i].work_time=0;
							  for(q=j;q<label_idle1;q++)
							  	{sort_idle_list_1[q]=sort_idle_list_1[q+1];}
							    sort_idle_list_1[label_idle1].myid=k;
								sort_idle_list_1[label_idle1].size=t;
						     }
					    }
				    }
	          }

	     }
     }  
    if(label_idle2!=0)
     { for(i=1;i<=label_busy2;i++)
	  	{if(sort_busy_list_2[i].work_time>=TIME_OUT)
      	      {  
                for(j=1;j<=label_idle2;j++)
                	{  
                       if(sort_busy_list_2[i].size==sort_idle_list_2[j].size)
                       	{ order_trans_rs485(mybox.myid,sort_idle_list_2[j].myid,1,2,1);
                          order_trans_rs485(mybox.myid,sort_busy_list_2[i].myid,1,2,0);
						  	{ k=sort_busy_list_2[i].myid;
                              t=sort_busy_list_2[i].size;
                              sort_busy_list_2[i].work_time=0;
							  for(q=j;q<label_idle2;q++)
							  	{sort_idle_list_2[q]=sort_idle_list_2[q+1];}
							    sort_idle_list_2[label_idle1].myid=k;
								sort_idle_list_2[label_idle1].size=t;
						     }
					    }
				    }
	          }

	     }
     }  
   // if(label_idle1==0&&label_idle2==0)break;//�޿��ж��� 
 
}

void unload_power(status_list_node *list_1,status_list_node *list_2)
{
 u8 i,j;
 s8 label_busy1,label_busy2;
 u16 glys;
  flag=1;
for(i=1;i<33;i++)inquiry_slave_status(i);
 label_busy2=sort_busynode_list(sort_busy_list_2,list_2);
	for(j=label_busy2;j>0;j--)
		{
           delay_time(1);
           glys=gonglvyinshu();
		   if(glys<95)break;
		   	order_trans_rs485(mybox.myid,sort_busy_list_2[j].myid,1,2,0);
        }
for(i=1;i<33;i++)inquiry_slave_status(i);
 label_busy1=sort_busynode_list(sort_busy_list_1,list_1);
	for(j=label_busy1;j>0;j--)
		{
           delay_time(1);
           glys=gonglvyinshu();
		   if(glys<95)break;
		   	order_trans_rs485(mybox.myid,sort_busy_list_2[j].myid,1,2,0);
		    
	    }
}
s8 sort_idlenode_list(idle_list *sort_idle_list,status_list_node *list)//�����������(��������С�ɴ�С���У����ؿ��нڵ����)
{
   u8 i,j=1,k,t,flag=0;
   s8 count=0;
   for(i=1;i<33;i++){sort_idle_list[i].myid=0;sort_idle_list[i].size=0;}
   for(i=1;i<33;i++){
   	              if(list[i].work_status==0&&list[i].myid!=0)
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
void sort_timenode_list(time_list *sort_time_list,status_list_node *list)//ʱ���������(������ʱ���ɴ�С����)
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
s8 sort_busynode_list(busy_list *sort_busy_list,status_list_node *list)//æµ�������(��������С��С��������)
{
   u8 i,j=1,k,t,w,flag=0;
   s8 count=0;
   for(i=1;i<33;i++){sort_busy_list[i].myid=0;sort_busy_list[i].size=0;}
   for(i=1;i<33;i++){
   	              if(list[i].work_status==1&&list[i].myid!=0)
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
   for(i=2;i<count;i++)
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
{ u32 i;
  for(i=0;i<time;i++) 
  { OSSemPost(Heartbeat);
    delay_ms(100);
    time++;
  }
}  //��ϵͳ����ʱ������time*150ms

void inquiry_slave_status(u8 id)   
  {  u8 *msg;
        u8 err;
   order_trans_rs485(mybox.myid,id,2,0,0);
   delay_us(10000);
   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/50,&err);
   if(err==OS_ERR_TIMEOUT){set_statuslist_1(id,0,2,0);set_statuslist_2(id,0,2,0);}//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
	  rs485_trans_status(msg);
	if(id==mybox.myid)
		{
	   set_statuslist_1(mystatus.myid,mystatus.size[0],mystatus.work_status[0],mystatus.work_time[0]);//����״̬��Ϣд��״̬��
       set_statuslist_2(mystatus.myid,mystatus.size[1],mystatus.work_status[1],mystatus.work_time[1]);
		}
	} //��ѯ�ӻ�״̬�����浽�ӻ�״̬���У�����id��Ҫ��ѯ�Ĵӻ���






/*******************����������غ���*****************************/

u16 power_computer()
{
        u16 i;
		u32 tempa,tempb;
		u16 adc_vx,adc_vmax=0,adc_ix,adc_imax=0;
		u8 phase_zhi;
		 float temp;

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
	  if(TIM3CH1_CAPTURE_STA&0X80)//���һ�βɼ�
		{
			tempa=TIM3CH1_CAPTURE_STA&0X3F;
			tempa*=65536;					//���ʱ���ܺ�
			tempa+=TIM3CH1_CAPTURE_VAL;		//�õ�TI1���ź�����ʱ��


			tempb=TIM3CH1_CAPTURE_STA&0X3F;
			tempb*=65536;					//���ʱ���ܺ�
			tempb+=TIM3CH1_CAPTURE_PHA;		//�õ�TI2 TI1������ʱ���ֵ����λ��ʱ��

	 		 if(tempb<=5000)			   //���Ը�������
			 {
			 	phase_zhi=tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			  if((10000<=tempb)&&(tempb<=15000))			 //���Ը��ط���
			 {
			 	phase_zhi=((tempb*360)/tempa)-180;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 if((5000<tempb)&&(tempb<10000))	   //���Ը�������
			 {
				/*��ʾ���Թ��ʷ���*/
			 	phase_zhi=180-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 if((15000<tempb)&&(tempb<20000))	   //���Ը��ط���
			 {
				/*��ʾ���Թ��ʷ���*/
			 	phase_zhi=360-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 wugongkvar=tempb;
			
			TIM3CH1_CAPTURE_STA=0;			//������һ�β���
			return gonglvshishu;
		}
	  return 0;

//�޹�����
}

u8 gonglvyinshu()
{
		u16 i;
		u32 tempa,tempb;
		u16 adc_vx,adc_vmax=0,adc_ix,adc_imax=0;
		u8 phase_zhi;
		 float temp;

		id_num=AT24CXX_ReadOneByte(0x0010);
		//key_idset();

		 for(i=0;i<120;i++)
	  	 {
	  	   adc_vx=Get_Adc_Average(ADC_CH1,10);
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
	  dianliuzhi=(u32)(60*temp-74);
	  adc_vmax=0;
	  adc_imax=0;
	   
	  if(TIM3CH1_CAPTURE_STA&0X80)//���һ�βɼ�
		{
			tempa=TIM3CH1_CAPTURE_STA&0X3F;
			tempa*=65536;					//���ʱ���ܺ�
			tempa+=TIM3CH1_CAPTURE_VAL;		//�õ�TI1���ź�����ʱ��


			tempb=TIM3CH1_CAPTURE_STA&0X3F;
			tempb*=65536;					//���ʱ���ܺ�
			tempb+=TIM3CH1_CAPTURE_PHA;		//�õ�TI2 TI1������ʱ���ֵ����λ��ʱ��

	 		 if(tempb<=5000)			   //���Ը�������
			 {
			 	phase_zhi=tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 else if(10000<=tempb&&tempb<=15000)			 //���Ը��ط���
			 {
			 	phase_zhi=((tempb*360)/tempa)-180;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 if(5000<tempb&&tempb<10000)	   //���Ը�������
			 {
				/*��ʾ���Թ��ʷ���*/
			 	phase_zhi=180-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			else if(15000<tempb&&tempb<20000)	   //���Ը��ط���
			 {
				/*��ʾ���Թ��ʷ���*/
			 	phase_zhi=360-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 wugongkvar=(uint16_t)(1.732*dianliuzhi*dianya_zhi*k*co[phase_zhi]);
			
			TIM3CH1_CAPTURE_STA=0;			//������һ�β���
		}
		 return gonglvshishu;
	}	 


void myled()
  {
//gonglvyinshu();
//key_idset();
//delay_ms(100);
 power_computer();
}


	 


