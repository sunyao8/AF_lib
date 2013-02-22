#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "485ANN.h"
#include "stm32f10x_tim.h"
u8 RS485_RX_BUF[64]; 		//接收缓冲,最大64个字节
//接收到的数据长度
u8 RS485_RX_CNT=0;  
//模式控制
 u16  dog_clock=10;

 OS_EVENT * RS485_MBOX;			//	rs485邮箱信号量
 OS_EVENT *Heartbeat;			 //心跳信号量

u8 cont=0;//用于更改主机号的记次数器

u8 token[33];//主机号令牌

box mybox;

u8 rs485buf[LEN],lon=LEN;

 void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM3, ENABLE);  //使能TIMx					 
}
 
 void TIM3_IRQHandler(void)   //TIM3中断
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{	  
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志
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
	#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();    
      #endif
 		
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收到数据
	{	 
	 			 
		 RS485_RX_BUF[RS485_RX_CNT++]=USART_ReceiveData(USART2); 	//读取接收到的数据
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='&'){RS485_RX_BUF[0]='&'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='*')
		{
				RS485_RX_CNT=0;
				OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);
		} 
	}  	
	#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntExit();  											 
#endif

} 

void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			//设置为发送模式
  	for(t=0;t<len;t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);		
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;				//设置为接收模式	

}

void initmybox(u8 id)//初始化自身信息
{  	 
  u8 i;
  
  for(i=1;i<33;i++)token[i]=0;//初始化令牌
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
void turn_master_id(u8 id)//改变当前整个系统中主机的ID号
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

void modfiy_token_array(u8 i,u8 j)//原主机位为0，新主机位为1
{
u8 k;
for(k=1;k<33;k++)
if(token[k]==1)
{token[k]=0;
break;
} 
token[i]=j;

}


 int rs485_trans_order(u8 *tx_r485)//解析由主机发送过来的信号，并发送给下位机
{ 
  
   if(mybox.myid==tx_r485[2])//判断是否是发给本机的信息
   	{
   	 mybox.source=tx_r485[1];
   	 mybox.send=tx_r485[3];
     mybox.relay=tx_r485[4];
     mybox.message=tx_r485[5];
 return 1;
   	}
   else return 0;
}

 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message)//主机程序，主机命令解析成RS485信息，发送给目的从机
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    rs485buf[0]='&';
	rs485buf[1]=source;
	rs485buf[2]=destination;
	rs485buf[3]=send;
	rs485buf[4]=relay;
	rs485buf[5]=message;
	rs485buf[6]='*';
	RS485_Send_Data(rs485buf,7);//发送5个字节
	OS_EXIT_CRITICAL();	
}




void Heartbeat_task(void *pdata)//master任务发送任务
{		// u8 key;
        u8 i=0;
		u8 err;
	//  u8 rs485buf[5];
	while(1)
	{
	OSSemPend(Heartbeat,0,&err);
		//key=KEY_Scan(0);
		//if(key==KEY_RIGHT)//KEY0按下,发送一次数据
		for(i=1;i<33;i++)
		{	
	       order_trans_rs485(mybox.myid,i,0,0,0);
		    delay_us(10000);
		  // LCD_ShowxNum(60+i*32,190,0,7,16,0X80);
		}		 
				   
	}
}

















