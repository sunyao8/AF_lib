#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "485ANN.h"
#include "stm32f10x_tim.h"
u8 RS485_RX_BUF[64]; 		//���ջ���,���64���ֽ�
//���յ������ݳ���
u8 RS485_RX_CNT=0;  
//ģʽ����
 u16  dog_clock=10;

 OS_EVENT * RS485_MBOX;			//	rs485�����ź���
 OS_EVENT *Heartbeat;			 //�����ź���

u8 cont=0;//���ڸ��������ŵļǴ�����

u8 token[33];//����������

box mybox;

u8 rs485buf[LEN],lon=LEN;

 void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx					 
}
 
 void TIM3_IRQHandler(void)   //TIM3�ж�
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
		{	  
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־
	if(mybox.master==0)	
		{
		
		if(dog_clock==0)
		   { 
		    LED0=!LED0;
			turn_master_id(mybox.myid);
			  cont++;
			}
			if(dog_clock>0)dog_clock--;
		 }
		}
   	OSIntExit();  
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
				OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);
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
  u8 i;
  
  for(i=1;i<33;i++)token[i]=0;//��ʼ������
 // if(ID==1){mybox.master=1;token[1]=1;}
  // if(ID!=1)mybox.master=0;
  mybox.master=0;
   token[1]=1;
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
u8 i,j,flag=0;
for(i=1;i<33;i++)
if(token[i]==1)
	{ 
	  flag=i+cont;
      if(id==(flag)){
        token[i]=0;
		token[id]=1;
		cont=0;
	  for(j=1;j<33;j++)
     {  order_trans_rs485(id,j,2,id,1);
	   delay_us(10000);
	  }
	 //  LED1=!LED1;
	   mybox.master=1;
	    OSTaskResume(MASTER_TASK_PRIO);
      	}
   }
}

void modfiy_token_array(u8 i,u8 j)//ԭ����λΪ0��������λΪ1
{
u8 k;
for(k=1;k<33;k++)
if(token[k]==1)
{token[k]=0;
break;
} 
token[i]=j;

}


 int rs485_trans_order(u8 *tx_r485)//�������������͹������źţ������͸���λ��
{ 
  
   if(mybox.myid==tx_r485[2])//�ж��Ƿ��Ƿ�����������Ϣ
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
    rs485buf[0]='&';
	rs485buf[1]=source;
	rs485buf[2]=destination;
	rs485buf[3]=send;
	rs485buf[4]=relay;
	rs485buf[5]=message;
	rs485buf[6]='*';
	RS485_Send_Data(rs485buf,7);//����5���ֽ�
	OS_EXIT_CRITICAL();	
}




void Heartbeat_task(void *pdata)//master����������
{		// u8 key;
        u8 i=0;
		u8 err;
	//  u8 rs485buf[5];
	while(1)
	{
	OSSemPend(Heartbeat,0,&err);
		//key=KEY_Scan(0);
		//if(key==KEY_RIGHT)//KEY0����,����һ������
		for(i=1;i<33;i++)
		{	
	       order_trans_rs485(mybox.myid,i,0,0,0);
		    delay_us(10000);
		  // LCD_ShowxNum(60+i*32,190,0,7,16,0X80);
		}		 
				   
	}
}

















