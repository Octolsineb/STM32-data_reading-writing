#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "sdram.h"
#include "usmart.h"
#include "mpu.h"
#include "mpu9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

// luyishi add 20200722
#include "GNSS_INS_Fusion.h"           /* Model's header file */
#include "rtwtypes.h"
#include "zero_crossing_types.h"

// shenxiangxiang add 20200730
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "sdram.h"
#include "w25qxx.h"
#include "nand.h"  
#include "mpu.h"
#include "sdmmc_sdcard.h"
#include "usmart.h"
#include "malloc.h"
#include "ftl.h"  
#include "ff.h"
#include "exfuns.h"
//#include "datacatch.h"
#include "data_reading.h"

/************************************************
 ALIENTEK 阿波罗STM32F746开发板 实验35
 MPU9250九轴传感器实验-HAL库函数版
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/


//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
	
{
	while((USART1->ISR&0X40)==0);	//循环发送,直到发送完毕   
    USART1->TDR=c;  
} 
//传送数据给匿名四轴地面站(V4版本)
//fun:功能字. 0X01~0X1C
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
//void usart1_niming_report(u8 fun,u8*data,u8 len)
//{
//	u8 send_buf[32];
//	u8 i;
//	if(len>28)return;	//最多28字节数据 
//	send_buf[len+3]=0;	//校验数置零
//	send_buf[0]=0XAA;	//帧头
//	send_buf[1]=0XAA;	//帧头
//	send_buf[2]=fun;	//功能字
//	send_buf[3]=len;	//数据长度
//	for(i=0;i<len;i++)send_buf[4+i]=data[i];			//复制数据
//	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//计算校验和	
//	for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
//}
//发送加速度传感器数据+陀螺仪数据(传感器帧)
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值 
//void mpu9250_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
//{
//	u8 tbuf[18]; 
//	tbuf[0]=(aacx>>8)&0XFF;
//	tbuf[1]=aacx&0XFF;
//	tbuf[2]=(aacy>>8)&0XFF;
//	tbuf[3]=aacy&0XFF;
//	tbuf[4]=(aacz>>8)&0XFF;
//	tbuf[5]=aacz&0XFF; 
//	tbuf[6]=(gyrox>>8)&0XFF;
//	tbuf[7]=gyrox&0XFF;
//	tbuf[8]=(gyroy>>8)&0XFF;
//	tbuf[9]=gyroy&0XFF;
//	tbuf[10]=(gyroz>>8)&0XFF;
//	tbuf[11]=gyroz&0XFF;
//	tbuf[12]=0;//因为开启MPL后,无法直接读取磁力计数据,所以这里直接屏蔽掉.用0替代.
//	tbuf[13]=0;
//	tbuf[14]=0;
//	tbuf[15]=0;
//	tbuf[16]=0;
//	tbuf[17]=0;
//	usart1_niming_report(0X02,tbuf,18);//传感器帧,0X02
//}	
//通过串口1上报结算后的姿态数据给电脑(状态帧)
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
//csb:超声波高度,单位:cm
//prs:气压计高度,单位:mm
//void usart1_report_imu(short roll,short pitch,short yaw,short csb,int prs)
//{
//	u8 tbuf[12];   	
//	tbuf[0]=(roll>>8)&0XFF;
//	tbuf[1]=roll&0XFF;
//	tbuf[2]=(pitch>>8)&0XFF;
//	tbuf[3]=pitch&0XFF;
//	tbuf[4]=(yaw>>8)&0XFF;
//	tbuf[5]=yaw&0XFF;
//	tbuf[6]=(csb>>8)&0XFF;
//	tbuf[7]=csb&0XFF;
//	tbuf[8]=(prs>>24)&0XFF;
//	tbuf[9]=(prs>>16)&0XFF;
//	tbuf[10]=(prs>>8)&0XFF;
//	tbuf[11]=prs&0XFF;
//	usart1_niming_report(0X01,tbuf,12);//状态帧,0X01
//}  
// 

//*******************************数据定义
typedef double real_T;
typedef float real32_T;
real32_T time;



FATFS fatsd;
FATFS fatflash;
FATFS fatnand;
FIL fileobj;
FRESULT fr;
FRESULT fr2;

UINT brs;

FIL objtxt;
FIL objtxt2;

	
u8 res=0;	

int reading_line_No=0;
  char buffer[32];	
  char buf[50];
	char buf1[50];
	char buf2[50];
	char buf3[50];
	char bufall[5060];

//*****************************


int main(void)
{
	//*********************************************************************初始化部分*************************************************************************
	u8 t=0,report=1;	            //默认开启上报
	u8 key;
	float pitch,roll,yaw; 	        //欧拉角
	short aacx,aacy,aacz;	        //加速度传感器原始数据
	short gyrox,gyroy,gyroz;        //陀螺仪原始数据 
	short temp;		                //温度 
    
    Cache_Enable();                 //打开L1-Cache
    MPU_Memory_Protection();        //保护相关存储区域
    HAL_Init();				        //初始化HAL库
    Stm32_Clock_Init(432,25,2,9);   //设置时钟,216Mhz 
    delay_init(216);                //延时初始化
	uart_init(500000);		        //串口初始化
	usmart_dev.init(108);		//初始化USMAR  
	LED_Init();		  			//初始化与LED连接的硬件接口   
	KEY_Init();                     //初始化按键
    SDRAM_Init();                   //初始化SDRAM
    LCD_Init();                     //LCD初始化
    MPU9250_Init();             	//初始化MPU9250
   	POINT_COLOR=RED;
//	LCD_ShowString(30,50,200,16,16,"Apollo STM32F4/F7"); 
//	LCD_ShowString(30,70,200,16,16,"MPU9250 TEST");	
//	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
//	LCD_ShowString(30,110,200,16,16,"2016/7/19");



  //  Cache_Enable();                 //打开L1-Cache
  //  MPU_Memory_Protection();        //保护相关存储区域
  //  HAL_Init();				        //初始化HAL库
  //  Stm32_Clock_Init(432,25,2,9);   //设置时钟,216Mhz 
  //  delay_init(216);                //延时初始化
//	uart_init(115200);		        //串口初始化
//	usmart_dev.init(108); 		    //初始化USMART
//    LED_Init();                     //初始化LED
 //   KEY_Init();                     //初始化按键
 //   SDRAM_Init();                   //初始化SDRAM
 //   LCD_Init();                     //初始化LCD
    W25QXX_Init();				    //初始化W25Q256
 	my_mem_init(SRAMIN);		    //初始化	内部内存池
	my_mem_init(SRAMEX);		    //初始化外部内存池
	my_mem_init(SRAMDTCM);		    //初始化CCM内存池 
   	POINT_COLOR=RED;
//	LCD_ShowString(30,50,200,16,16,"Apollo STM32F4/F7"); 
//	LCD_ShowString(30,70,200,16,16,"FATFS TEST");	
//	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
//	LCD_ShowString(30,110,200,16,16,"2016/7/15");	 	 
//	LCD_ShowString(30,130,200,16,16,"Use USMART for test");	      
 	while(SD_Init())//检测不到SD卡
	{
//		LCD_ShowString(30,150,200,16,16,"SD Card Error!");
//		delay_ms(1000);					
//		LCD_ShowString(30,150,200,16,16,"Please Check! ");
		delay_ms(1000);
		LED0_Toggle;//DS0闪烁
	}
    FTL_Init();
 	exfuns_init();							//为fatfs相关变量申请内存				 
  	f_mount(&fatsd,"0:",1); 					//挂载SD卡 1表示立即挂载
 	res=f_mount(&fatflash,"1:",1); 				//挂载FLASH.	
	res=f_mount(&fatnand,"2:",1); 				//挂载NAND FLASH.	
	
	
//	fr=f_open(&objtxt,"0:/GNSS_INS_data.txt",FA_OPEN_EXISTING | FA_READ);
//	   f_lseek(&objtxt, 0);

 	 
	while(mpu_dmp_init())         
    {   
//		LCD_ShowString(30,130,200,16,16,"MPU9250 Error");
//		delay_ms(200);
//		LCD_Fill(30,130,239,130+16,WHITE);
 		delay_ms(2000);
		LED0_Toggle;//DS0闪烁 
    }
//    LCD_ShowString(30,130,200,16,16,"MPU9250 OK");
//	LCD_ShowString(30,150,200,16,16,"KEY0:UPLOAD ON/OFF");
//    POINT_COLOR=BLUE;     //设置字体为蓝色
//    LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");	 
// 	LCD_ShowString(30,200,200,16,16," Temp:    . C");	
// 	LCD_ShowString(30,220,200,16,16,"Pitch:    . C");	
// 	LCD_ShowString(30,240,200,16,16," Roll:    . C");	 
// 	LCD_ShowString(30,260,200,16,16," Yaw :    . C");	
		
		GNSS_INS_Fusion_initialize();
		fr=f_open(&objtxt,"0:/GNSS_INS_data.txt",FA_OPEN_EXISTING | FA_READ);
		fr2=f_open(&objtxt2,"0:/YAW_INS.txt",FA_OPEN_EXISTING | FA_WRITE);
		while(reading_line_No<350000)
		{
		
		
		
	//*********************************************************循环读取，并且赋值接口********************************************************
     
		 data_read();
		

	//*********************************************************计算过程**********************************************************************

				GNSS_INS_Fusion_step();
  		GNSS_INS_Fusion_terminate();

	//********************************************************结果写入新的txt中*********************************************
      answer_write();
			
			reading_line_No++;
		}



	

	
	
	
	//*******************************
//        key=KEY_Scan(0);
//		if(key==KEY0_PRES)
//		{
//            report=!report;
//			if(report)LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");
//			else LCD_ShowString(30,170,200,16,16,"UPLOAD OFF");
//		}
//        if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
//        {
//            temp=MPU_Get_Temperature();	//得到温度值
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
//  			if(report)mpu9250_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//发送加速度+陀螺仪原始数据
//			if(report)usart1_report_imu((int)(roll*100),(int)(pitch*100),(int)(yaw*100),0,0);
//			if((t%10)==0)
//			{ 
//				if(temp<0)
//				{
//					LCD_ShowChar(30+48,200,'-',16,0);		//显示负号
//					temp=-temp;		//转为正数
//				}else LCD_ShowChar(30+48,200,' ',16,0);		//去掉负号 
//				LCD_ShowNum(30+48+8,200,temp/100,3,16);		//显示整数部分	    
//				LCD_ShowNum(30+48+40,200,temp%10,1,16);		//显示小数部分 
//				temp=pitch*10;
//				if(temp<0)
//				{
//					LCD_ShowChar(30+48,220,'-',16,0);		//显示负号
//					temp=-temp;		//转为正数
//				}else LCD_ShowChar(30+48,220,' ',16,0);		//去掉负号 
//				LCD_ShowNum(30+48+8,220,temp/10,3,16);		//显示整数部分	    
//				LCD_ShowNum(30+48+40,220,temp%10,1,16);		//显示小数部分 
//				temp=roll*10;
//				if(temp<0)
//				{
//					LCD_ShowChar(30+48,240,'-',16,0);		//显示负号
//					temp=-temp;		//转为正数
//				}else LCD_ShowChar(30+48,240,' ',16,0);		//去掉负号 
//				LCD_ShowNum(30+48+8,240,temp/10,3,16);		//显示整数部分	    
//				LCD_ShowNum(30+48+40,240,temp%10,1,16);		//显示小数部分 
//				temp=yaw*10;
//				if(temp<0)
//				{
//					LCD_ShowChar(30+48,260,'-',16,0);		//显示负号
//					temp=-temp;		//转为正数
//				}else LCD_ShowChar(30+48,260,' ',16,0);		//去掉负号 
//				LCD_ShowNum(30+48+8,260,temp/10,3,16);		//显示整数部分	    
//				LCD_ShowNum(30+48+40,260,temp%10,1,16);		//显示小数部分  
//				t=0;
//				LED0_Toggle;//DS0闪烁 
//			}
//		}
//        t++;
		


 LED0_Toggle;
	while(1)         
    {   
 		delay_ms(2000);
		LED0_Toggle;//DS0闪烁 
}

}


