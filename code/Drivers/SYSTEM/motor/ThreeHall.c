//############################################################
// FILE: ThreeHall.c
// Created on: 2017��1��18��
// Author: XQ
// summary: ThreeHall
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//�޸�����:2017/1/24
//�汾��V17.3-1
//Author-QQ: 616264123
//�������QQȺ��314306105
//############################################################
#include "ThreeHall.h"

extern   Hall   Hall_Three;

  uint16_t   HallK1=355; 		//Offset filter coefficient K1: 0.05/(T+0.05);
  uint16_t   HallK2=669 ;  	//Offset filter coefficient K2: T/(T+0.05);
  
	int16_t   init_angle=-8980 ;  //   4�Լ� 42BL��� 13620  �����ĳ�ʼ�Ƕ�  0---65536  0---360��

 void  ThreeHallPara_init(void )
{
   Hall_Three.Hall_num[0]=5;
   Hall_Three.Hall_num[1]=1;
   Hall_Three.Hall_num[2]=3;
   Hall_Three.Hall_num[3]=2;
   Hall_Three.Hall_num[4]=6;
   Hall_Three.Hall_num[5]=4;

   Hall_Three.Hall_angle[0]=65535;  // 1
   Hall_Three.Hall_angle[1]=10923;  // 1/6
   Hall_Three.Hall_angle[2]=21845;  // 1/3
   Hall_Three.Hall_angle[3]=32678;  // 1/2
   Hall_Three.Hall_angle[4]=43691;  // 2/3
   Hall_Three.Hall_angle[5]=54613;  // 5/6
   	
   Hall_Three.step_angle_error =0;   //  *10  IQ(1/6) /(  x/(60 * 10e6 *Fkg))     10 7374 1824       20 9715 0000       2097152   349525 *6000      60��/(60 000000��0.0001)
   Hall_Three.Poles=4;
   Hall_Three.initial_angle=init_angle;  // 182 = 1 ��   -50*182=9100   +78*182 =13620
	 Hall_Three.speed_coeff=(1000*60)/(2*Hall_Three.Poles ); // 2�������һ�νǶȲ�ֵ 1000/2ms=500   =7500
}


void ThreeHallanglecale(void)  // һ��PWM����ִ��һ��
{
	   if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)==1) 
     Hall_Three.HallUVW[0]=1;
		 else
		 Hall_Three.HallUVW[0]=0; 
   	 if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)==1) 
     Hall_Three.HallUVW[1]=1;
		 else
		 Hall_Three.HallUVW[1]=0; 
		 if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15)==1) 
     Hall_Three.HallUVW[2]=1;
		  else
		 Hall_Three.HallUVW[2]=0; 
	 
	  Hall_Three.Hall_State = Hall_Three.HallUVW[0] +(Hall_Three.HallUVW[1]<<1) +(Hall_Three.HallUVW[2]<<2);

    if ( Hall_Three.Hall_State!=Hall_Three.OldHall_State )
      {
    	  Hall_Three.HallLX_State=Hall_Three.Hall_State + (Hall_Three.OldHall_State<<4) ;
  
    	  switch (Hall_Three.HallLX_State )
    	        {
    	         case 0x45:
    	          {
    	             Hall_Three.step_angle_error=Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[0]-Hall_Three.angleIQ;    	           
    	          	 if( Hall_Three.step_angle_error>65536 )
								   Hall_Three.step_angle_error-=65536;
									 if( Hall_Three.step_angle_error<0 )
								   Hall_Three.step_angle_error+=65536;    	            
                   Hall_Three.Move_State=1;
    	           }
    	         break;
    	         case 0x15:
    	           {
    	              Hall_Three.step_angle_error=  Hall_Three.Hall_angle[1]+Hall_Three.angleIQ-Hall_Three.Hall_angle[1];
    	          	  if( Hall_Three.step_angle_error>65536 )
								  	Hall_Three.step_angle_error-=65536;
									  if( Hall_Three.step_angle_error<0 )
								  	Hall_Three.step_angle_error+=65536;
										Hall_Three.Move_State=2;
    	           }
    	         break;
    	         case 0x51:
    	           {
    	          	  Hall_Three.step_angle_error=  Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[1]- Hall_Three.angleIQ;
    	          	  if( Hall_Three.step_angle_error>65536 )
								  	Hall_Three.step_angle_error-=65536;
										if( Hall_Three.step_angle_error<0 )
								  	Hall_Three.step_angle_error+=65536;    	           
                    Hall_Three.Move_State=1;
    	           }
    	         break;
    	         case  0x31:
    	            {
    	            	Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[2]; 
    	          	  if( Hall_Three.step_angle_error>65536 )
								  	Hall_Three.step_angle_error-=65536;
									  if( Hall_Three.step_angle_error<0 )
								  	Hall_Three.step_angle_error+=65536;    	             
									  Hall_Three.Move_State=2;
    	            }
    	         break;
    	         case 0x13:
    	            {
    	          	  Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[2]- Hall_Three.angleIQ ;
    	          	  if( Hall_Three.step_angle_error>65536 )
								  	Hall_Three.step_angle_error-=65536;
										if( Hall_Three.step_angle_error<0 )
								  	Hall_Three.step_angle_error+=65536; 
    	          	  Hall_Three.Move_State=1;
    	            }
    	         break;
    	         case 0x23:
    	            {
    	              Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[3];
    	          	  if( Hall_Three.step_angle_error>65536 )
								  	Hall_Three.step_angle_error-=65536;
										if( Hall_Three.step_angle_error<0 )
								  	Hall_Three.step_angle_error+=65536;
                    Hall_Three.Move_State=2;
    	            }
    	          break;
    	         case 0x32:
    	            {
    	          	  Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[3]- Hall_Three.angleIQ ;
    	          	  if( Hall_Three.step_angle_error>65536 )
								  	Hall_Three.step_angle_error-=65536;
									  if( Hall_Three.step_angle_error<0 )
								  	Hall_Three.step_angle_error+=65536;
                    Hall_Three.Move_State=1;
    	            }
    	         break;
    	         case 0x62:
    	            {
    	              Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[4] ;
    	              if( Hall_Three.step_angle_error>65536 )
								  	Hall_Three.step_angle_error-=65536;
										if( Hall_Three.step_angle_error<0 )
								  	Hall_Three.step_angle_error+=65536; 
									  Hall_Three.Move_State=2;
    	            }
    	          break;
    	         case 0x26:
    	            {
    	          	  Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[4]- Hall_Three.angleIQ;
    	          	  if( Hall_Three.step_angle_error>65536 )
								  	Hall_Three.step_angle_error-=65536;
										if( Hall_Three.step_angle_error<0 )
								  	Hall_Three.step_angle_error+=65536; 
									  Hall_Three.Move_State=1;
    	            }
    	         break;
    	         case 0x46:
    	            {
    	              Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[5] ;
     	          	  if( Hall_Three.step_angle_error>65536 )
								  	Hall_Three.step_angle_error-=65536;
									  if( Hall_Three.step_angle_error<0 )
								  	Hall_Three.step_angle_error+=65536;
                    Hall_Three.Move_State=2;
    	            }
    	          break;
    	         case 0x64:
    	            {
    	          	  Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[5]- Hall_Three.angleIQ;
    	          	  if( Hall_Three.step_angle_error>65536 )
								  	Hall_Three.step_angle_error-=65536;
										if( Hall_Three.step_angle_error<0 )
								  	Hall_Three.step_angle_error+=65536;
                    Hall_Three.Move_State=1;
    	            }
    	         break;
    	         case 0x54:
    	            {
    	              Hall_Three.step_angle_error= Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[0];
    	          	  if( Hall_Three.step_angle_error>65536 )
								  	Hall_Three.step_angle_error-=65536;
										if( Hall_Three.step_angle_error<0 )
								  	Hall_Three.step_angle_error+=65536;
                    Hall_Three.Move_State=2;
    	            }
    	          break;

    	           default:
    	          	{
    	          		Hall_Three.ele_angleIQ=0;
										Hall_Three.Move_State=0;
    	          	}
    	           break;
    	         } 
		  Hall_Three.Speed_countFitter= _IQ10mpy(HallK2, Hall_Three.Speed_countFitter)+_IQ10mpy(HallK1,  Hall_Three.Speed_count);								  
			Hall_Three.Speed_count_old=Hall_Three.Speed_countFitter;
      Hall_Three.step_angle = Hall_Three.step_angle_error/Hall_Three.Speed_countFitter;
		  Hall_Three.step_angleFitter= _IQ10mpy(HallK2, Hall_Three.step_angleFitter)+_IQ10mpy(HallK1,  Hall_Three.step_angle);	
  		Hall_Three.Speed_count= 0;        
     }

   else  if ( Hall_Three.Hall_State==Hall_Three.OldHall_State )
     {
    	 Hall_Three.Speed_count++;   
       if( Hall_Three.Speed_count>=2000 )
    	 {
    		 Hall_Three.Speed_count=0;
    		 Hall_Three.Speed_RPM= 0 ;
    		 Hall_Three.step_angleFitter=0;	
				 Hall_Three.Move_State=0;
						switch (Hall_Three.Hall_State )
						{
							case 0x5:
							{
							Hall_Three.angleIQ = Hall_Three.Hall_angle[0]+5462;
							}
						  break;
						  case 0x1:
							{
								Hall_Three.angleIQ = Hall_Three.Hall_angle[1]+5462;
							}
						  break;
						  case 0x3:
							{
							Hall_Three.angleIQ = Hall_Three.Hall_angle[2]+5462;
							}
						  break;
						  case 0x2:
							{
								Hall_Three.angleIQ = Hall_Three.Hall_angle[3]+5462;
							}
						  break;
						  case 0x6:
							{
							Hall_Three.angleIQ = Hall_Three.Hall_angle[4]+5462;
							}
						  break;
						  case 0x4:
							{
								Hall_Three.angleIQ = Hall_Three.Hall_angle[5]+5462;
							}
						  break;

						  default:
							{
								Hall_Three.ele_angleIQ=0;
							}
						  break;
						 }
       }
   }		 
      if ( Hall_Three.Move_State==1 )
       {
        Hall_Three.angleIQ = Hall_Three.angleIQ + Hall_Three.step_angleFitter ;
       }
       else if ( Hall_Three.Move_State==2 )
       {
        Hall_Three.angleIQ = Hall_Three.angleIQ - Hall_Three.step_angleFitter ;
       }

     if( Hall_Three.angleIQ> 65536)
  	  Hall_Three.angleIQ-=65536;
     else if( Hall_Three.angleIQ<0)
  	  Hall_Three.angleIQ+=65536;
     
     Hall_Three.ele_angleIQ = Hall_Three.angleIQ-16384 +Hall_Three.initial_angle ;

     if( Hall_Three.ele_angleIQ> 65536)
  	  Hall_Three.ele_angleIQ-=65536;
     else if( Hall_Three.ele_angleIQ<0)
  	  Hall_Three.ele_angleIQ+=65536;

      Hall_Three.OldHall_State=Hall_Three.Hall_State ;
}

void Hall_Three_Speedcale(void)  // 2msִ��һ��
{
  if( Hall_Three.Move_State==1)
  {
	   Hall_Three.Speed_ele_angleIQ =Hall_Three.ele_angleIQ -Hall_Three.old_ele_angleIQ ;
	   if( Hall_Three.Speed_ele_angleIQ <0)
		 Hall_Three.Speed_ele_angleIQ+= 65536;
  }
  else  if( Hall_Three.Move_State==2)
  {
  	 Hall_Three.Speed_ele_angleIQ =Hall_Three.old_ele_angleIQ -Hall_Three.ele_angleIQ;
  	 if( Hall_Three.Speed_ele_angleIQ < 0)
  	 Hall_Three.Speed_ele_angleIQ+=65536; 	 
  }
	else    // Hall_Three.Move_State=0 
		 Hall_Three.Speed_ele_angleIQ=0;
 	 Hall_Three.Speed_ele_angleIQFitter= _IQ10mpy(HallK1, Hall_Three.Speed_ele_angleIQFitter)+_IQ10mpy(HallK2,  Hall_Three.Speed_ele_angleIQ);
   Hall_Three.Speed_RPM = (Hall_Three.Speed_ele_angleIQ*Hall_Three.speed_coeff)>>16; // ���Ƕ� 2pi��һȦ 65536
	 Hall_Three.old_ele_angleIQ = Hall_Three.ele_angleIQ ;
}
 


//===========================================================================
// No more.
//===========================================================================
