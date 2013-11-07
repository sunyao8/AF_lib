#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "485ANN.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "key.h"
#include "math.h"
#include "stm32_dsp.h"

/***********************************************************************/
#define CPT_LL                                                    '^'
#define CONTROL                                                '/'

u16 RS485_RX_BUF[64]; 		//���ջ���,���64���ֽ�
//���յ������ݳ���
u8 RS485_RX_CNT=0;  
//ģʽ����
 u16  dog_clock=10;

 OS_EVENT * RS485_MBOX,* RS485_STUTAS_MBOX;			//	rs485�����ź���
 OS_EVENT *Heartbeat;			 //�����ź���
OS_EVENT *master_led_task;
//OS_EVENT * sub_machine1_open;		//��λ�������ź�
//OS_EVENT * sub_machine1_close;		//��λ�������ź�
//OS_EVENT * sub_machine2_open;		//��λ�������ź�
//OS_EVENT * sub_machine2_close;		//��λ�������ź�

OS_EVENT *scan_slave;

u8 cont=0;//���ڸ��������ŵļǴ�����
u32 life_time_1=0;//�ӻ�1����ʱ����м����
u32 life_time_2=0;//�ӻ�2����ʱ����м����
u32 idle_time=0;//�����������ݵ�ʱ��
u32 scan_time=0;//�������ڴ��±����ӻ��ĵ�ʱ��
u16 m1_opentime,m2_opentime,m1_closetime,m2_closetime;//�ӻ�ʹ�ñ���
u8 true_worktime1_flag=0,true_worktime2_flag=0;//�ӻ�ʹ�ñ��������ڱ�ʶ��λ������͸�е�ʱ���
u8 turn_flag=1;//����ʹ�ñ���
s8 turn_label_idle=0;//����ʹ�ñ���

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

//u8 rs485buf[LEN_control];//���Ϳ�����Ϣ
u8 rs485buf[LEN_control];//���Ϳ�����Ϣ
u8 statusbuf[LEN_status];//����״̬��Ϣ


u8 alarm_lock=0;
u8 phase_zhi=0;
/****************************************************************/
#define NPT 512            /* NPT = No of FFT point*/
#define FFT_NM NPT/2
#define PI2  6.28318530717959
long lBUFIN_V[NPT];         /* Complex input vector */
long lBUFOUT_V[FFT_NM];        /* Complex output vector */
long lBUFIN_I[NPT];         /* Complex input vector */
long lBUFOUT_I[FFT_NM];        /* Complex output vector */
double angle[3]; 



/**********************�����޹����� ����*************************************/


/************************************************************/
u16 wugong_95,wugong_computer;

extern u8 id_num;
extern u8 grafnum,tempshuzhi,gonglvshishu;
extern u16 dianya_zhi,	wugongkvar,k;
extern u32	dianliuzhi;
s8 L_C_flag;//�������Ա�׼����
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

	if(mybox.master==1)	
				{
                                   scan_time++;
					if(scan_time==65530)scan_time=0;			   

			       }
	
	
//		 if (mystatus.work_status[0]==1&&true_worktime1_flag==1)  //����ʱ��ļ�ʱ
		 		 if (mystatus.work_status[0]==1)  //����ʱ��ļ�ʱ
		    {  
		        life_time_1++;
		       if(life_time_1==5)
			   { mystatus.work_time[0]++;
			     life_time_1=0;
				  }
			        if(mystatus.work_time[0]==37)mystatus.work_time[0]=39;
				  if(mystatus.work_time[0]==254)mystatus.work_time[0]=0;
			   }
		 	
		 //if(mystatus.work_status[1]==1&&true_worktime2_flag==1)
		 if(mystatus.work_status[1]==1)
		 	{ 
		 	life_time_2++;
              if(life_time_2==5)
			  	{  mystatus.work_time[1]++;
			       life_time_2=0;
              	         }   
                   if(mystatus.work_time[1]==37)mystatus.work_time[1]=39;
			 if(mystatus.work_time[1]==254)mystatus.work_time[1]=0;	   
			    
		     }
		  // if (mystatus.work_status[0]==0&&true_worktime1_flag==0)  //����ʱ������
		   		   if (mystatus.work_status[0]==0)  //����ʱ������
		    {   mystatus.work_time[0]=0;
		   }
		   // if (mystatus.work_status[1]==0&&true_worktime2_flag==0)  //����ʱ������
		    		    if (mystatus.work_status[1]==0)  //����ʱ������
		    {   mystatus.work_time[1]=0;
			}

			if(mybox.master==1)
		       {  idle_time++;
			if(idle_time==65535)idle_time=0;
			}

		


	}
   	OSIntExit();  
}

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

void initmybox()//��ʼ��������Ϣ
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

void turn_master_id(u8 id)//�ı䵱ǰ����ϵͳ��������ID��
{
   u8 flag=0;
	{ 
	  flag=cont;
      if(id==(flag)){
	//delay_time(2);
         mybox.master=1;
	    OSTaskResume(MASTER_TASK_PRIO);
		cont=1;
	  }
	 //  LED1=!LED1;
	  
      }
   }






 int rs485_trans_order(u8 *tx_r485)//�������������͹������źţ������͸���λ��
{
 	         
 
if(tx_r485[8]==CPT_LL)
{
    dianya_zhi=comp_16(tx_r485[1],tx_r485[2]);
  dianliuzhi=comp_16(tx_r485[3],tx_r485[4]);
  wugongkvar=comp_16(tx_r485[5],tx_r485[6]);
  gonglvshishu=tx_r485[7];
return 0;

}



if(tx_r485[8]==CONTROL)

{
   if(mybox.myid==tx_r485[2])//�ж��Ƿ��Ƿ�����������Ϣ���ǹ㲥��Ϣ
   	{
   	 mybox.source=tx_r485[1];
   	 mybox.send=tx_r485[3];
     mybox.relay=tx_r485[4];
     mybox.message=tx_r485[5];
     return 1;
   	}
   
}

return 0;

}

 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr)//���������������������RS485��Ϣ�����͸�Ŀ�Ĵӻ�
{   OS_CPU_SR cpu_sr=0;
   OS_ENTER_CRITICAL();
    
   {  
  if(ctr==CONTROL)
    {
      rs485buf[0]='&';//Э��ͷ
	rs485buf[1]=source;
	rs485buf[2]=destination;
	rs485buf[3]=send;
	rs485buf[4]=relay;
	rs485buf[5]=message;
	rs485buf[6]=0;
	rs485buf[7]=gonglvshishu;	
	rs485buf[8]=ctr;
	rs485buf[9]='*';//Э��β
	RS485_Send_Data(rs485buf,10);//����5���ֽ�
	   	if(destination==source){mybox.send=send;slave_control(relay, message);}//�����Ϣ�������Լ�

  }
	
if(ctr==CPT_LL )

		{
      rs485buf[0]='&';//Э��ͷ	
	rs485buf[1]=(dianya_zhi & (uint16_t)0x00FF);
	rs485buf[2]=((dianya_zhi & (uint16_t)0xFF00)>>8);
	rs485buf[3]=(dianliuzhi& (uint16_t)0x00FF);
	rs485buf[4]=((dianliuzhi& (uint16_t)0xFF00)>>8);
	rs485buf[5]=(wugongkvar& (uint16_t)0x00FF);
	rs485buf[6]=((wugongkvar& (uint16_t)0xFF00)>>8);
	rs485buf[7]=gonglvshishu;	
	rs485buf[8]=ctr;	
	rs485buf[9]='*';//Э��β
	RS485_Send_Data(rs485buf,10);//����5���ֽ�
	  // 	if(destination==source){mybox.send=send;slave_control(relay, message);}//�����Ϣ�������Լ�

    	}
//#endif
}
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
	       order_trans_rs485(mybox.myid,0,0,0,0,CPT_LL);
		    delay_us(10000);
		  // LCD_ShowxNum(60+i*32,190,0,7,16,0X80);
		}	
}










void led_on_off(u8 on_off,u8 j) //����ֵ1 Ϊ��led ��0Ϊ�ر�led
{
}


/*****************************������Ϣ����********************************************/


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

 void status_trans_rs485_dis(status_box *mystatus)//�ӻ�����
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    statusbuf[0]='&';
	statusbuf[1]='#';
	statusbuf[2]=mystatus->myid;
	statusbuf[3]=mystatus->size[0];
	statusbuf[4]=mystatus->size[1];
	statusbuf[5]=mystatus->work_status[0];
	statusbuf[6]=mystatus->work_status[1];

	statusbuf[7]='*';
	RS485_Send_Data(statusbuf,8);//����10���ֽ�
	OS_EXIT_CRITICAL();	
}

 void status_trans_rs485_RT(status_box *mystatus)//�ӻ�����
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    statusbuf[0]='&';
	statusbuf[1]='#';

	statusbuf[2]='*';
	RS485_Send_Data(statusbuf,3);//����10���ֽ�
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


void offset_idlepower()  //���ʲ����������������� �޹����� �������� ���ж���
{
 u8 i,j;
 u8 label_idle1=0,label_idle2=0;
  turn_flag=1;
  //led_on_off(IDLE_NODE_LCD_LOCK,0);
  led_on_off(IDLE_NODE_LCD_LOCK,0);
for(i=1;i<=slave[0];i++)inquiry_slave_status(slave[i]);
  led_on_off(ALL_NODE_LCD_UNLOCK,0);
 label_idle1=sort_idlenode_list(sort_idle_list_1,system_status_list_1);
  label_idle2=sort_idlenode_list(sort_idle_list_2,system_status_list_2);
 
/***********************************************************************/
//label_idle1=0;//ʵ�� ֻͶ�еڶ����� ����Ʒ����ȥ����仰
/************************************************************************/

	for(j=1;j<=label_idle1;j++)
		{

		/*	if(done_list1_flag==0)
                            {

				   for(i=1;i<=label_idle1;i++)
				   	    {
                                          idle_done_nodelist_1[sort_idle_list_1[i].myid]=0;

				           }
				   done_list1_flag=1;
			       }	
                   */
						 
			delay_time(1);
		
		              if(gonglvshishu>90)break;
		                 else{
                    if((wugong_computer)>=(sort_idle_list_1[j].size))
			             { ///  if(idle_done_nodelist_1[sort_idle_list_1[j].myid]==0)    ///
			                   {                   
                                           led_on_off(NODE_LCD_LOCK_BASE,sort_idle_list_1[j].myid);//1000ms
				//	order_trans_rs485(mybox.myid,sort_idle_list_1[j].myid,Sub_Order,1,1);
			order_trans_rs485(mybox.myid,sort_idle_list_1[j].myid,1,1,1,CONTROL);

						///		idle_done_nodelist_1[sort_idle_list_1[j].myid]=1;   ///
										 
		
					    }
					}
				///	done_count_1++;///
				///	if(done_count_1==label_idle1)///
				///		{done_count_1=0;done_list1_flag=0;}///
		   	                  }

				myled();  //120ms		 
		   }

 	if(gonglvshishu<90)
 		{
	for(j=1;j<=label_idle2;j++)
		{
		         /*             if(done_list2_flag==0)
                            {

				   for(i=1;i<=label_idle2;i++)
				   	    {
                                          idle_done_nodelist_2[sort_idle_list_2[i].myid]=0;

				           }
				   done_list2_flag=1;
			       }
		    */ 
			delay_time(1);
		

		   if(gonglvshishu>90)break;
		   else{
                          if((wugong_computer)>=(sort_idle_list_2[j].size))
                          	            { /// if(idle_done_nodelist_2[sort_idle_list_2[j].myid]==0)
                          	                {     
                                        led_on_off(NODE_LCD_LOCK_BASE,sort_idle_list_2[j].myid);
					//	  order_trans_rs485(mybox.myid,sort_idle_list_2[j].myid,Sub_Order,2,1);///
					order_trans_rs485(mybox.myid,sort_idle_list_2[j].myid,1,2,1,CONTROL);
                                                idle_done_nodelist_2[sort_idle_list_2[j].myid]=1;
										
                          	                }
					///	  done_count_2++;
					///	  	if(done_count_2==label_idle2)
					///	  	{done_count_2=0;done_list2_flag=0;}
						  
						  }
		   	 }

			myled();
	    }

 		}

}

void turn_power(status_list_node *list_1,status_list_node *list_2)//���������������ȶ������󣬽�����������ݺ���
{
u8 i,j,k,t,q;
u8 label_busy1,label_busy2;
 if(turn_flag==1)
   {
   			led_on_off(IDLE_NODE_LCD_LOCK,0);
   for(i=1;i<=slave[0];i++)inquiry_slave_status(slave[i]); 
   turn_label_idle=turn_idlenode_list(turn_idle_list,list_1,list_2);//�õ����ж���
   }  
   turn_flag=0;

if(turn_label_idle!=0)
{	      led_on_off(BUSY_NODE_LCD_LCOK,0);
   for(i=1;i<=slave[0];i++)inquiry_slave_status(slave[i]);   			
    label_busy1=sort_busynode_list(sort_busy_list_1,list_1);//ˢ��list_1��busy���ÿ�������ڵ�Ĺ���ʱ��
    label_busy2=sort_busynode_list(sort_busy_list_2,list_2);//ˢ��list_2��busy���ÿ�������ڵ�Ĺ���ʱ��

      for(i=1;i<=label_busy1;i++)
	  	{ if(sort_busy_list_1[i].work_time>=TIME_OUT)
      	      {  
				for(j=1;j<=turn_label_idle;j++)
                	{  
                       if(sort_busy_list_1[i].size==turn_idle_list[j].size)
                       	{ order_trans_rs485(mybox.myid,turn_idle_list[j].myid,1,turn_idle_list[j].group,1,CONTROL);delay_us(10000);
                             order_trans_rs485(mybox.myid,sort_busy_list_1[i].myid,1,1,0,CONTROL);
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
                                           break;//��ֹ��ѭ���ж�i���ظ�ƥ��Ͷ��

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
                       	{ order_trans_rs485(mybox.myid,turn_idle_list[j].myid,1,turn_idle_list[j].group,1,CONTROL);delay_us(10000);
                             order_trans_rs485(mybox.myid,sort_busy_list_2[i].myid,1,2,0,CONTROL);delay_us(10000);
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
                                           break;//��ֹ��ѭ���ж�i���ظ�ƥ��Ͷ��

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
//label_busy1=0;//ʵ�� ֻͶ�еڶ����� ����Ʒ����ȥ����仰
/************************************************************************/

	for(j=1;j<=label_busy1&&label_busy1>0;)
		{
                     delay_time(1);
		   if(gonglvshishu<95)break;
		   else {	
						if((wugong_computer-wugong_95)>sort_busy_list_1[label_busy1].size)
				   {
                             led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_1[label_busy1].myid);                                         											
				   order_trans_rs485(mybox.myid,sort_busy_list_1[label_busy1].myid,1,1,0,CONTROL);
				  // delay_ms(3500);
				   label_busy1--;
                              

				}

						if((wugong_computer-wugong_95)<=sort_busy_list_1[j].size)
				   {
	                             led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_1[j].myid);                                         														   
				   order_trans_rs485(mybox.myid,sort_busy_list_1[j].myid,1,1,0,CONTROL);
				 // delay_ms(3500);
                              j++;
				}

		          }
		   myled();//���㹦�ʣ���ѹ��������ʾ�����ֿ�
	    }

if(gonglvshishu>95)
 {
	for(j=1;j<=label_busy2&&label_busy2>0;)
		{
                        delay_time(1);
		   if(gonglvshishu<95)break;
		   else{	                               
                                	if((wugong_computer-wugong_95)>sort_busy_list_2[label_busy2].size)
				   {
				          led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_2[label_busy2].myid);                                         											
				   order_trans_rs485(mybox.myid,sort_busy_list_2[label_busy2].myid,1,2,0,CONTROL);
				 // delay_ms(3500);
				   label_busy2--;
                                                         
				}

			   if((wugong_computer-wugong_95)<=sort_busy_list_2[j].size)
		   	       {
				          led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_2[j].myid);                                         											
				   order_trans_rs485(mybox.myid,sort_busy_list_2[j].myid,1,2,0,CONTROL);
				  // delay_ms(3500);
				   j++;
			   }
                         				
   

		           }
		  myled();//���㹦�ʣ���ѹ��������ʾ�����ֿ�
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
		   if(L_C_flag==1)break;
		   else {	
                            				if((wugong_computer)>sort_busy_list_1[label_busy1].size)
				                            { 									
								  led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_1[label_busy1].myid);
								order_trans_rs485(mybox.myid,sort_busy_list_1[label_busy1].myid,1,1,0,CONTROL);
				                           //   delay_ms(3500);
				                                label_busy1--;
				                             }

                                                            if((wugong_computer)<=sort_busy_list_1[j].size)
			                                     { 
			                                     led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_1[j].myid);
                          			                  order_trans_rs485(mybox.myid,sort_busy_list_1[j].myid,1,1,0,CONTROL);
					                      //    delay_ms(3500);
											  j++;
	                                                    }
		          }

		   myled();//���㹦�ʣ���ѹ��������ʾ�����ֿ�
	    }

if(L_C_flag==0)
 {
	for(j=1;j<=label_busy2&&label_busy2!=0;)
		{
                            delay_time(1);
		   if(L_C_flag==1)break;
		   else{	
                                                            if((wugong_computer)>sort_busy_list_2[label_busy2].size)
				   {
			                                     led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_2[label_busy2].myid);                                                          								
				   order_trans_rs485(mybox.myid,sort_busy_list_2[label_busy2].myid,1,2,0,CONTROL);
				//  delay_ms(3500);
				   label_busy2--;

				}

			 if((wugong_computer)<=sort_busy_list_2[j].size)
			{
                     led_on_off(NODE_LCD_LOCK_BASE,sort_busy_list_2[j].myid);                                      														   
			order_trans_rs485(mybox.myid,sort_busy_list_2[j].myid,1,2,0,CONTROL);
			//delay_ms(3500);	
			j++;

                     }

		   }

	myled();//���㹦�ʣ���ѹ��������ʾ�����ֿ�
        }


}
}

u8 sort_idlenode_list(idle_list *sort_idle_list,status_list_node *list)//�����������(��������С�ɴ�С���У����ؿ��нڵ����)
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


//δ���
u8 offset_idlenode_one_list(offset_node *offset_idle_list,idle_list *sort_idle_list_1,idle_list *sort_idle_list_2)//�����������(��������С�ɴ�С���У����ؿ��нڵ����)
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






/**************����һ����ж��к͵ڶ�����ж������һ����������� ��***************************/


u8 turn_idlenode_list(turn_node *turn_idle_list,status_list_node *list_1,status_list_node *list_2)//�����������(��������С�ɴ�С���У����ؿ��нڵ����)
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
						if(flag==0)flag=1;//���û�п��нڵ�
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
						if(flag==0)flag=1;//���û�п��нڵ�
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
u8 sort_busynode_list(busy_list *sort_busy_list,status_list_node *list)//æµ�������(������ʱ���С�ɴ�С����)
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


u8 sort_busynode_list_asc(busy_list *sort_busy_list,status_list_node *list)//æµ�������(������������С�ɴ�С����)
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
}  //��ϵͳ����ʱ������time*10ms

u8 inquiry_slave_status(u8 id)   
  {  u8 *msg;
        u8 err;
			if(id==mybox.myid)
		{
	   set_statuslist_1(mystatus.myid,mystatus.size[0],mystatus.work_status[0],mystatus.work_time[0]);//����״̬��Ϣд��״̬��
       set_statuslist_2(mystatus.myid,mystatus.size[1],mystatus.work_status[1],mystatus.work_time[1]);
	   return 1;
		}

   order_trans_rs485(mybox.myid,id,2,0,0,CONTROL);
  // delay_us(10000);
   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/50,&err);
   if(err==OS_ERR_TIMEOUT){set_statuslist_1(id,0,2,0);set_statuslist_2(id,0,2,0);return 0;}//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
	{  rs485_trans_status(msg);return 1;}

} //��ѯ�ӻ�״̬�����浽�ӻ�״̬���У�����id��Ҫ��ѯ�Ĵӻ���






/*******************����������غ���*****************************/

void gonglvyinshu()
{
		u16 adc_vx=0,adc_vmax=0,adc_imax=0;
		//u16 adc_ix;
		u16 k=100;//����ϵ��
		 float adcv,adci,temp,X,Y,Mag;
	        u16 i=0,flag_v=5,flag_i=5;
	         int32_t lX=0,lY=0;



		id_num=AT24CXX_ReadOneByte(0x0010);

/*	 for(i=0;i<600;i++)
	  {
		 adc_ix=Get_Adc_Average(ADC_Channel_4,2);
		 if(adc_ix>adc_imax)
		 adc_imax=adc_ix;
	  }	
*/
//if(adc_imax>=1750)
        	{
        for(i=0;i<NPT;i++)
        	{
		adci=(Get_Adc_Average(ADC_Channel_4,4)-1520)/10;
		adcv=(Get_Adc_Average(ADC_Channel_1,4)-1520)/10;                //10
		adcv=(Get_Adc_Average(ADC_Channel_1,4)-1520)/10;
		adci=(Get_Adc_Average(ADC_Channel_4,4)-1520)/10;
		lBUFIN_V[i]=((short)adcv) << 16;
		lBUFIN_I[i]=((short)adci) << 16;		
		
        	}

}
/*
if(adc_imax<1750)
{
        for(i=0;i<NPT;i++)
        	{
		adci=(Get_Adc_Average(ADC_Channel_4,4)-1520)/7;
		adcv=(Get_Adc_Average(ADC_Channel_1,4)-1520)/7;                //10
		adcv=(Get_Adc_Average(ADC_Channel_1,4)-1520)/7;
		adci=(Get_Adc_Average(ADC_Channel_4,4)-1520)/7;
		lBUFIN_V[i]=((short)adcv) << 16;
		lBUFIN_I[i]=((short)adci) << 16;		
		
        	}

}

*/

			 allphase(lBUFIN_V,lBUFIN_I);

		
          cr4_fft_256_stm32(lBUFOUT_V, lBUFIN_V, NPT);
	   cr4_fft_256_stm32(lBUFOUT_I, lBUFIN_I, NPT);

			 
		 lX  = (lBUFOUT_V[flag_v] << 16) >> 16;
                 lY  = (lBUFOUT_V[flag_v] >> 16);
				angle[0]=atan2(lY,lX);

		lX  = (lBUFOUT_I[flag_i] << 16) >> 16;
                 lY  = (lBUFOUT_I[flag_i] >> 16);
				   {
                   X    = FFT_NM* ((float)lX) /32768;
                   Y    = FFT_NM* ((float)lY) /32768;
                    Mag = sqrt(X*X + Y*Y)/FFT_NM/1.414;
					k=1000/k;
                   dianliuzhi= (u32)(Mag * 65536)/k;
                   }
				angle[1]=atan2(lY,lX);
				
				angle[2]=((angle[1]-angle[0])*360)/PI2-90;
				if(angle[2]>0.0)
			{
				if(angle[2]<180)
					{
                           		gonglvshishu=(u8)abs(sin((angle[1]-angle[0]))*100+0.5);
								L_C_flag=1;
				       }
								if(angle[2]>=180&&angle[2]<=360)
					{
                           		gonglvshishu=(u8)abs(sin((angle[1]-angle[0]))*100-0.5);
								L_C_flag=0;
				       }

				

                    }
				
				else if(angle[2]<=0.0)
			{
					if((angle[2]>=-180.0&&angle[2]<-360.0))
					{
			     gonglvshishu=(u8)abs(sin((angle[1]-angle[0]))*100+0.5);
				 L_C_flag=1;

				  }

					if((angle[2]>-180.0&&angle[2]<=0.0))
					{
			     gonglvshishu=(u8)abs(sin((angle[1]-angle[0]))*100-0.5);
				 L_C_flag=0;

				  }	
		
			}
	
		 for(i=0;i<500;i++)
	  	 {
	  	 adc_vx=Get_Adc_Average(ADC_Channel_1,3);
		  // adc_vx=Get_Adc(ADC_Channel_1);
		   if(adc_vx>adc_vmax)
		   adc_vmax=adc_vx;
	  	 }
/*		 
	   for(i=0;i<500;i++)
	  {
		 adc_ix=Get_Adc_Average(ADC_CH4,3);
		 if(adc_ix>adc_imax)
		 adc_imax=adc_ix;
	  }

*/
             

	  temp=((float)adc_vmax*(3.3/4096))*1000;
	dianya_zhi=((u16)(5*temp-6005))/10;
 	  adc_vmax=0;
	  adc_imax=0;
			 wugongkvar=(uint16_t)((1.732*dianliuzhi*dianya_zhi*abs(cos((angle[1]-angle[0]))*100))/100000);
			wugong_95= (uint16_t)((17.32*dianliuzhi*dianya_zhi*31)/100000);//����������0.95ʱ�ģ��޹����
			//wugong_computer=(uint16_t)((17.32*dianliuzhi*dianya_zhi*sin((angle[1]-angle[0]))*100/100000));
			wugong_computer=(uint16_t)((17.32*dianliuzhi*dianya_zhi*abs(cos((angle[1]-angle[0]))*100))/100000);
                    //wugongkvar=wugong_computer;
			
		
		

//�޹�����
}



void allphase(long *V,long *I)
{
int i=0;
for(i=0;i<=NPT/2-1;i++)
{
V[i]=(i+1)*V[i];
I[i]=(i+1)*I[i];
}
for(i=NPT/2;i<NPT-1;i++)
{
V[i]=(NPT-(i+1))*V[i];
I[i]=(NPT-(i+1))*I[i];

}

for(i=0;i<NPT/2-1;i++)
{
V[i+NPT/2]=V[i]+V[i+NPT/2];
I[i+NPT/2]=I[i]+I[i+NPT/2];

}

for(i=0;i<=NPT/2-1;i++)
{
V[i]=V[NPT/2-1+i];
I[i]=I[NPT/2-1+i];

}
}


void temperature()   //�������¶ȼ��
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
gonglvyinshu();//���㹦�ʣ���ѹ��������ʾ�����ֿ�
temperature();
key_lcd();
delay_ms(50);//û����ʱ����������
}

void Alarm(void)
{
/*
	   if((tempshuzhi>=70||dianya_zhi>=440||dianya_zhi<=340)&&alarm_lock==0)
	   	{   
                                        if(mybox.master==1)//��������֪ͨ��ʱ��Ϣ
								  {
								       delay_time(80);
								  }   
				GPIO_SetBits(GPIOA,GPIO_Pin_0);
					delay_us(100000);
                    GPIO_SetBits(GPIOA,GPIO_Pin_8);
		       set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],2,2,0,0);
		       LIGHT(mystatus.work_status[0],mystatus.work_status[1]);//���³�̬
		       alarm_lock=1;

	   }
	   
	/*	   if((dianya_zhi>=440||dianya_zhi<=340)&&mybox.master==1)//����ǹ�ѹ��Ƿѹ���� ��������������ѭ��
		   	{
                             while(1)
					{   gonglvyinshu();//���㹦�ʣ���ѹ��������ʾ�����ֿ�
                                        temperature();
                                           key_idset();//��������ʾ����
                                             delay_time(1);
						delay_ms(100);//û����ʱ����������

					if(dianya_zhi<440&&dianya_zhi>340)break;						 

				        }

		      }
		      
��������
*/
/*
		  if(tempshuzhi<70&&dianya_zhi<440&&dianya_zhi>340&&alarm_lock==1)
	         {

			   if(mybox.master==1)//��������֪ͨ��ʱ��Ϣ
								  {
								       delay_time(80);
								  }
		    set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,0,0,0);
		    LIGHT(mystatus.work_status[0],mystatus.work_status[1]);//�ָ���̬
		    alarm_lock=0;
	 	}

	
*/	 

}
	 
void key_lcd()
{
key_idset();//��������ʾ����
Alarm();//�Ƿ���Ҫ����
 LIGHT(mystatus.work_status[0],mystatus.work_status[1]);//ˢָʾ�ƣ������ʾ������·�����˲� ����ɾ��
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






