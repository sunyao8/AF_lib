//#include <stm32f10x_lib.h>
#include "key.h"
#include "delay.h"
#include "ht1621.h"
#include "led.h"
//#include "exti.h"
#include "24cxx.h" 	
static u8 m=1;
static u8 grafnum=1;
u8 zhongduan_flag=1;
u8 id_num=0;
u8 grafnum,tempshuzhi,vernum=101,hguestnum=222,gonglvshishu=0;
u16 dianya_zhi=0,wugongkvar=0;
u32	dianliuzhi=0;
u16 k=10;//电流系数
//////////////////////////////////////////////////////////////////////////////////	 
//本程序为控制器设计，未经许可，不得复制外传
//实验板栋达电子V3.0-1
//KEY 代码 PA11为显示板设置按键；PA12为手动投切开关	   
//修改日期:2013/3/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 济宁市栋达电子科技有限公司 2013-2023
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  
								    
//按键初始化函数
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<2;     //使能PORTA时钟
	GPIOA->CRH&=0XFFF00FFF;	//PA11 PA12设置成输入	  
	GPIOA->CRH|=0X00088000; 
	GPIOA->ODR|=1<11;		// 上拉
	GPIOA->ODR|=1<12;		  //上拉
} 

void key_idset(void)
{
	
	u8 h;
	if((KEY0==0)&&m)
	{
	   delay_us(10000);
	   m=0;
	   if(KEY0==0)
	   while(KEY0==0)
	   	{
	   	   delay_us(10000);
		   h++;
		   if(h>=250)break;
	   
	   	}
			   if(h>=250)
			   {		
					zhongduan_flag=0;
						
			   		Clera_lcd();
		
			   		Graf_setid(id_num);
			   }
			   else
				   {  
				     if(zhongduan_flag==1)
				      	{
					  		grafnum++;
					  		if(grafnum>6)grafnum=1;
					  
					    	  switch(grafnum)
								{				 
									case 1:	//显示功率因数和电压值
										Clera_lcd();
										Graf_con_u(gonglvshishu,dianya_zhi);
										break;
									case 2:	//显示电流
										Clera_lcd();
										Graf_cuirrent(dianliuzhi);
										break;
									case 3:	//显示无功功率	 
										Clera_lcd();
										Graf_qkvar(wugongkvar);
										break;
									case 4:	//显示温度 
										Clera_lcd();
										Graf_temp(tempshuzhi);
										break;
					
									case 5:	//显示ID 
										Clera_lcd();
										Graf_id(hguestnum,id_num);
										break;
					
									case 6:	//显示VER 
										Clera_lcd();
										Graf_ver(vernum);
										break;
					
								}
					 	}
						if(zhongduan_flag==0)
				      	{
					  		id_num++;
					  		if(id_num>32)id_num=0;
							Clera_lcd();
	   						Graf_setid(id_num);
							AT24CXX_WriteOneByte(0x0010,id_num);
						}
				   }
	
	}
	else if(KEY0==1)
		{
		 	m=1;
			delay_us(10000);
			 while(KEY0==1)
			 {
		   	   delay_us(10000);
			   h++;
			   if(h>=250)break;
	   
	   		 }
			   if(h>=250)
				 {
						  zhongduan_flag=1;
					  	  switch(grafnum)
							{				 
								case 1:	//显示功率因数和电压值
									Clera_lcd();
									Graf_con_u(gonglvshishu,dianya_zhi);
									break;
								case 2:	//显示电流
									Clera_lcd();
									Graf_cuirrent(dianliuzhi);
									break;
								case 3:	//显示无功功率	 
									Clera_lcd();
									Graf_qkvar(wugongkvar);
									break;
								case 4:	//显示温度 
									Clera_lcd();
									Graf_temp(tempshuzhi);
									break;
				
								case 5:	//显示ID 
									Clera_lcd();
									Graf_id(hguestnum,id_num);
									break;
				
								case 6:	//显示VER 
									Clera_lcd();
									Graf_ver(vernum);
									break;
				
							}	
				 }

		}
}




















