		
		
#include "led.h"

#include "ff.h"  

#include "stdlib.h"
#include "string.h"
#include "GNSS_INS_Fusion.h"
  
  char buffer_gnss_data[512];
  char buffer[32];	
 extern FIL objtxt;
 extern	UINT brs;
 extern	FIL objtxt2;
 extern	FRESULT fr2;
 extern	FRESULT fr;
	
  char buf[50];
	char buf1[50];
	char buf2[50];
	char buf3[50];
	char bufall[5060];
  
extern int reading_line_No;
int i=0,j=0,k=0;

typedef double real_T;
typedef float real32_T;

real32_T time;



void data_read(void)
{
	//fr=f_open(&objtxt,"0:/GNSS_INS_data.txt",FA_OPEN_EXISTING | FA_READ);
	//*********************************************************循环读取数据********************************************************

	  unsigned int reading_position,blank_number,buffer_writing_position;
	  LED0_Toggle;
	  f_lseek(&objtxt, j);
    f_read(&objtxt,buffer_gnss_data,512,&brs);
    
	  reading_position=0;	
	  blank_number=0;
	
	while(reading_position<512){
		buffer_writing_position=0;
		while(1){
				buffer[buffer_writing_position]=buffer_gnss_data[reading_position];
				buffer_writing_position++;
				reading_position++;
			  
			  if(buffer_gnss_data[reading_position]==0x09||buffer_gnss_data[reading_position]==0x0D||buffer_gnss_data[reading_position]==0x20) break;
		}
		switch(blank_number)
		{
			case 0:
				time=atof(buffer); blank_number++;	break;
			case 1:
				GNSS_INS_Fusion_U.IMU_ACC[0]=atof(buffer); blank_number++;break;
			case 2:
				GNSS_INS_Fusion_U.IMU_ACC[1]=atof(buffer); blank_number++;break;
			case 3:
		    GNSS_INS_Fusion_U.IMU_ACC[2]=atof(buffer); blank_number++;break;
			case 4:
				GNSS_INS_Fusion_U.IMU_Gyro[0]=atof(buffer); blank_number++;break;
			case 5:
				GNSS_INS_Fusion_U.IMU_Gyro[1]=atof(buffer); blank_number++;break;
			case 6:
				GNSS_INS_Fusion_U.IMU_Gyro[2]=atof(buffer); blank_number++;break;
			case 7:
				GNSS_INS_Fusion_U.HeadingAngle_GNSS=atof(buffer); blank_number++;break;
			case 8:
				GNSS_INS_Fusion_U.Latitude_GNSS_Int=atof(buffer); blank_number++;break;
			case 9:
				GNSS_INS_Fusion_U.Longitude_GNSS_Int=atof(buffer); blank_number++;break;
			case 10:
				GNSS_INS_Fusion_U.Latitude_GNSS_Dec=atof(buffer); blank_number++;break;
			case 11:
				GNSS_INS_Fusion_U.Longitude_GNSS_Dec=atof(buffer); blank_number++;break;
			case 12:
				GNSS_INS_Fusion_U.Pos_RMS_G[0]=atof(buffer); blank_number++;break;
			case 13:
				GNSS_INS_Fusion_U.Pos_RMS_G[1]=atof(buffer); blank_number++;break;
			case 14:
				GNSS_INS_Fusion_U.VehicleSpeed=atof(buffer); blank_number++;break;
			case 15:
			 GNSS_INS_Fusion_U. Quality_GNSS=atof(buffer); blank_number++;break;
			case 16:
			  GNSS_INS_Fusion_U.Vel_Level_GNSS=atof(buffer); blank_number++;break;
			case 17:
			  GNSS_INS_Fusion_U.Course_GNSS=atof(buffer); blank_number++;break;
			case 18:
			 GNSS_INS_Fusion_U.SteeringAngleValid=atof(buffer); blank_number++;break;
			case 19:
			  GNSS_INS_Fusion_U.SteeringAngle=atof(buffer); blank_number++;break;
			case 20:
			  GNSS_INS_Fusion_U.Heading_RMS_GNSS=atof(buffer); blank_number++;break;
			case 21:
			  GNSS_INS_Fusion_U.Soln_SVs_GNSS=atof(buffer); blank_number++;break;
			case 22:
			  GNSS_INS_Fusion_U.GNSS_Height=atof(buffer); blank_number++;break;
			case 23:
				GNSS_INS_Fusion_U.Soln_SVs_Ante2_GNSS=atof(buffer); blank_number++;break;
			case 24:
			  GNSS_INS_Fusion_U.GNSS_Vel_Vertical=atof(buffer); blank_number++;break;
			case 25:
			 GNSS_INS_Fusion_U.GNSS_Vel_Latency=atof(buffer); blank_number++;break;
			case 26:
			  GNSS_INS_Fusion_U.Heading_time=atof(buffer); blank_number++;break;
			case 27:
				GNSS_INS_Fusion_U.Pos_time=atof(buffer); blank_number++;break;
			case 28:
			  GNSS_INS_Fusion_U.Vel_time=atof(buffer); blank_number++;break;
			case 29:
			  GNSS_INS_Fusion_U.Pos_double[0]=atof(buffer); blank_number++;break;
			case 30:
			  GNSS_INS_Fusion_U.Pos_double[1]=atof(buffer); blank_number++;break;
			case 31:
			  GNSS_INS_Fusion_U.Pos_double[2]=atof(buffer); blank_number++;break;
			case 32:
			  GNSS_INS_Fusion_U.Lati_Dec_single=atof(buffer); blank_number++;break;
			case 33:
			  GNSS_INS_Fusion_U.Longi_Dec_single=atof(buffer); blank_number++;break;
			case 34:
			  GNSS_INS_Fusion_U.GNSS_Height_RMS=atof(buffer);blank_number++;break;
			case 35:
			  GNSS_INS_Fusion_U.IMU_TEMP=atof(buffer); blank_number++;break;
			break;
			
      default:
	    break;
		}
	memset(buffer,0,sizeof(buffer));	
		
	reading_position++;
  if(buffer_gnss_data[reading_position]==0x0D||buffer_gnss_data[reading_position]==0x0A) break;	
	
	}
   //printf("reading position is %d \r\n ",reading_position);
	//printf("this is line %d \r\n",cqwe);
  // printf("**********************************\r\n");

		j=j+reading_position;
}


		
void answer_write(void)
{
	sprintf(buf, "%.10lf \t",GNSS_INS_Fusion_Y.Yaw_INS);
	strcat(bufall,buf);
	memset(buf,0,sizeof(buf));
	sprintf(buf1, "%.10lf \t",GNSS_INS_Fusion_Y.Pos_new_Dec_single_lever[0]);
	strcat(bufall,buf1);
	memset(buf1,0,sizeof(buf1));
	sprintf(buf2, "%.10lf \t",GNSS_INS_Fusion_Y.Pos_new_Dec_single_lever[1]);
	strcat(bufall,buf2);
	memset(buf2,0,sizeof(buf2));
	sprintf(buf3, "%.10lf \r\n",GNSS_INS_Fusion_Y.Pos_new_Dec_single_lever[2]);
	strcat(bufall,buf3);
	memset(buf3,0,sizeof(buf3));
	if(strlen(bufall)>4096){
				f_lseek(&objtxt2,f_size(&objtxt2));
				fr2=f_write(&objtxt2,bufall,strlen(bufall),&brs);
				f_sync(&objtxt2);
				memset(bufall,0,sizeof(bufall));
        }

}