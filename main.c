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
 ALIENTEK ������STM32F746������ ʵ��35
 MPU9250���ᴫ����ʵ��-HAL�⺯����
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/


//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_char(u8 c)
	
{
	while((USART1->ISR&0X40)==0);	//ѭ������,ֱ���������   
    USART1->TDR=c;  
} 
//�������ݸ������������վ(V4�汾)
//fun:������. 0X01~0X1C
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
//void usart1_niming_report(u8 fun,u8*data,u8 len)
//{
//	u8 send_buf[32];
//	u8 i;
//	if(len>28)return;	//���28�ֽ����� 
//	send_buf[len+3]=0;	//У��������
//	send_buf[0]=0XAA;	//֡ͷ
//	send_buf[1]=0XAA;	//֡ͷ
//	send_buf[2]=fun;	//������
//	send_buf[3]=len;	//���ݳ���
//	for(i=0;i<len;i++)send_buf[4+i]=data[i];			//��������
//	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//����У���	
//	for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
//}
//���ͼ��ٶȴ���������+����������(������֡)
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ 
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
//	tbuf[12]=0;//��Ϊ����MPL��,�޷�ֱ�Ӷ�ȡ����������,��������ֱ�����ε�.��0���.
//	tbuf[13]=0;
//	tbuf[14]=0;
//	tbuf[15]=0;
//	tbuf[16]=0;
//	tbuf[17]=0;
//	usart1_niming_report(0X02,tbuf,18);//������֡,0X02
//}	
//ͨ������1�ϱ���������̬���ݸ�����(״̬֡)
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
//csb:�������߶�,��λ:cm
//prs:��ѹ�Ƹ߶�,��λ:mm
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
//	usart1_niming_report(0X01,tbuf,12);//״̬֡,0X01
//}  
// 

//*******************************���ݶ���
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
	//*********************************************************************��ʼ������*************************************************************************
	u8 t=0,report=1;	            //Ĭ�Ͽ����ϱ�
	u8 key;
	float pitch,roll,yaw; 	        //ŷ����
	short aacx,aacy,aacz;	        //���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;        //������ԭʼ���� 
	short temp;		                //�¶� 
    
    Cache_Enable();                 //��L1-Cache
    MPU_Memory_Protection();        //������ش洢����
    HAL_Init();				        //��ʼ��HAL��
    Stm32_Clock_Init(432,25,2,9);   //����ʱ��,216Mhz 
    delay_init(216);                //��ʱ��ʼ��
	uart_init(500000);		        //���ڳ�ʼ��
	usmart_dev.init(108);		//��ʼ��USMAR  
	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�   
	KEY_Init();                     //��ʼ������
    SDRAM_Init();                   //��ʼ��SDRAM
    LCD_Init();                     //LCD��ʼ��
    MPU9250_Init();             	//��ʼ��MPU9250
   	POINT_COLOR=RED;
//	LCD_ShowString(30,50,200,16,16,"Apollo STM32F4/F7"); 
//	LCD_ShowString(30,70,200,16,16,"MPU9250 TEST");	
//	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
//	LCD_ShowString(30,110,200,16,16,"2016/7/19");



  //  Cache_Enable();                 //��L1-Cache
  //  MPU_Memory_Protection();        //������ش洢����
  //  HAL_Init();				        //��ʼ��HAL��
  //  Stm32_Clock_Init(432,25,2,9);   //����ʱ��,216Mhz 
  //  delay_init(216);                //��ʱ��ʼ��
//	uart_init(115200);		        //���ڳ�ʼ��
//	usmart_dev.init(108); 		    //��ʼ��USMART
//    LED_Init();                     //��ʼ��LED
 //   KEY_Init();                     //��ʼ������
 //   SDRAM_Init();                   //��ʼ��SDRAM
 //   LCD_Init();                     //��ʼ��LCD
    W25QXX_Init();				    //��ʼ��W25Q256
 	my_mem_init(SRAMIN);		    //��ʼ��	�ڲ��ڴ��
	my_mem_init(SRAMEX);		    //��ʼ���ⲿ�ڴ��
	my_mem_init(SRAMDTCM);		    //��ʼ��CCM�ڴ�� 
   	POINT_COLOR=RED;
//	LCD_ShowString(30,50,200,16,16,"Apollo STM32F4/F7"); 
//	LCD_ShowString(30,70,200,16,16,"FATFS TEST");	
//	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
//	LCD_ShowString(30,110,200,16,16,"2016/7/15");	 	 
//	LCD_ShowString(30,130,200,16,16,"Use USMART for test");	      
 	while(SD_Init())//��ⲻ��SD��
	{
//		LCD_ShowString(30,150,200,16,16,"SD Card Error!");
//		delay_ms(1000);					
//		LCD_ShowString(30,150,200,16,16,"Please Check! ");
		delay_ms(1000);
		LED0_Toggle;//DS0��˸
	}
    FTL_Init();
 	exfuns_init();							//Ϊfatfs��ر��������ڴ�				 
  	f_mount(&fatsd,"0:",1); 					//����SD�� 1��ʾ��������
 	res=f_mount(&fatflash,"1:",1); 				//����FLASH.	
	res=f_mount(&fatnand,"2:",1); 				//����NAND FLASH.	
	
	
//	fr=f_open(&objtxt,"0:/GNSS_INS_data.txt",FA_OPEN_EXISTING | FA_READ);
//	   f_lseek(&objtxt, 0);

 	 
	while(mpu_dmp_init())         
    {   
//		LCD_ShowString(30,130,200,16,16,"MPU9250 Error");
//		delay_ms(200);
//		LCD_Fill(30,130,239,130+16,WHITE);
 		delay_ms(2000);
		LED0_Toggle;//DS0��˸ 
    }
//    LCD_ShowString(30,130,200,16,16,"MPU9250 OK");
//	LCD_ShowString(30,150,200,16,16,"KEY0:UPLOAD ON/OFF");
//    POINT_COLOR=BLUE;     //��������Ϊ��ɫ
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
		
		
		
	//*********************************************************ѭ����ȡ�����Ҹ�ֵ�ӿ�********************************************************
     
		 data_read();
		

	//*********************************************************�������**********************************************************************

				GNSS_INS_Fusion_step();
  		GNSS_INS_Fusion_terminate();

	//********************************************************���д���µ�txt��*********************************************
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
//            temp=MPU_Get_Temperature();	//�õ��¶�ֵ
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
//  			if(report)mpu9250_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���ͼ��ٶ�+������ԭʼ����
//			if(report)usart1_report_imu((int)(roll*100),(int)(pitch*100),(int)(yaw*100),0,0);
//			if((t%10)==0)
//			{ 
//				if(temp<0)
//				{
//					LCD_ShowChar(30+48,200,'-',16,0);		//��ʾ����
//					temp=-temp;		//תΪ����
//				}else LCD_ShowChar(30+48,200,' ',16,0);		//ȥ������ 
//				LCD_ShowNum(30+48+8,200,temp/100,3,16);		//��ʾ��������	    
//				LCD_ShowNum(30+48+40,200,temp%10,1,16);		//��ʾС������ 
//				temp=pitch*10;
//				if(temp<0)
//				{
//					LCD_ShowChar(30+48,220,'-',16,0);		//��ʾ����
//					temp=-temp;		//תΪ����
//				}else LCD_ShowChar(30+48,220,' ',16,0);		//ȥ������ 
//				LCD_ShowNum(30+48+8,220,temp/10,3,16);		//��ʾ��������	    
//				LCD_ShowNum(30+48+40,220,temp%10,1,16);		//��ʾС������ 
//				temp=roll*10;
//				if(temp<0)
//				{
//					LCD_ShowChar(30+48,240,'-',16,0);		//��ʾ����
//					temp=-temp;		//תΪ����
//				}else LCD_ShowChar(30+48,240,' ',16,0);		//ȥ������ 
//				LCD_ShowNum(30+48+8,240,temp/10,3,16);		//��ʾ��������	    
//				LCD_ShowNum(30+48+40,240,temp%10,1,16);		//��ʾС������ 
//				temp=yaw*10;
//				if(temp<0)
//				{
//					LCD_ShowChar(30+48,260,'-',16,0);		//��ʾ����
//					temp=-temp;		//תΪ����
//				}else LCD_ShowChar(30+48,260,' ',16,0);		//ȥ������ 
//				LCD_ShowNum(30+48+8,260,temp/10,3,16);		//��ʾ��������	    
//				LCD_ShowNum(30+48+40,260,temp%10,1,16);		//��ʾС������  
//				t=0;
//				LED0_Toggle;//DS0��˸ 
//			}
//		}
//        t++;
		


 LED0_Toggle;
	while(1)         
    {   
 		delay_ms(2000);
		LED0_Toggle;//DS0��˸ 
}

}


