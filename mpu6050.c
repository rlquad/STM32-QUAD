#include "mpu6050.h"
#include "mpu_setup.h"
#include "stdio.h"
#include <math.h>

int16_t gyro[3],acc[3];

int t1,t2,t3,t4,arm;
int flag1=0,flag2=0,flag3=0,flag4=0,flag5=0;
uint32_t t1_one,t2_one,t3_one,t4_one,arm_one;



void TIM2_IRQHandler(void)
{
    
///////////////////////////////////////////////////////////////////////////////////
//ARMING SWITCH
///////////////////////////////////////////////////////////////////////////////////
if(TIM_GetITStatus(TIM2,TIM_IT_CC2)==SET)
{
  TIM_ClearITPendingBit(TIM2,TIM_IT_CC2);
  if(flag5==0)
    {
      flag5 = 1;
      arm_one = TIM_GetCounter(TIM2);
    }
  else if(flag5==1)
    {
         if(TIM_GetCounter(TIM2)-arm_one<3000)
    {
        flag5 = 0;
        arm = TIM_GetCounter(TIM2)-arm_one;

       }
      else
      {

        arm_one = TIM_GetCounter(TIM2);
      }
    }
}
////////////////////////////////////////////////////////////////////////////////////
//PITCH
////////////////////////////////////////////////////////////////////////////////////
if(TIM_GetITStatus(TIM2,TIM_IT_CC4)==SET)
{
    TIM_ClearITPendingBit(TIM2,TIM_IT_CC4);
  if(flag2==0)
    {
      flag2 = 1;
      t2_one = TIM_GetCounter(TIM2);
    }
    else if(flag2==1)
    {
         if(TIM_GetCounter(TIM2)-t2_one<2100)
       {
        flag2 = 0;
        t2 = (int)(TIM_GetCounter(TIM2)- t2_one - 1500)*pitch_coeff;

       }
      else
      {

        t2_one = TIM_GetCounter(TIM2);
      }
    }

}
////////////////////////////////////////////////////////////////////////////////////
//YAW
////////////////////////////////////////////////////////////////////////////////////

if(TIM_GetITStatus(TIM2,TIM_IT_CC1)==SET)
{
    TIM_ClearITPendingBit(TIM2,TIM_IT_CC1);
  if(flag4==0)
    {
      flag4 = 1;
      t4_one = TIM_GetCounter(TIM2);
    }
    else if(flag4==1)
    {
         if(TIM_GetCounter(TIM2)-t4_one<2100)
       {
        flag4 = 0;
        t4 = (int)(TIM_GetCounter(TIM2)-t4_one - 1500)*yaw_coeff;

       }
      else
      {

        t4_one = TIM_GetCounter(TIM2);
      }
    }
}

}


void TIM1_CC_IRQHandler(void)
{
    
////////////////////////////////////////////////////////////////////////////////////////
//THROTTLE
///////////////////////////////////////////////////////////////////////////////////////
  if(TIM_GetITStatus(TIM1,TIM_IT_CC1)==SET)
{
    TIM_ClearITPendingBit(TIM1,TIM_IT_CC1);
  if(flag3==0)
    {
      flag3 = 1;
      t3_one = TIM_GetCounter(TIM2);
    }
    else if(flag3==1)
    {
         if(TIM_GetCounter(TIM2)-t3_one<2100)
       {
        flag3 = 0;
        t3 = (int)(TIM_GetCounter(TIM2)- t3_one );

       }
      else
      {

        t3_one = TIM_GetCounter(TIM2);
      }
    }
}

////////////////////////////////////////////////////////////////////////////////////////
//ROLL
///////////////////////////////////////////////////////////////////////////////////////
if(TIM_GetITStatus(TIM1,TIM_IT_CC3)==SET)
{
    TIM_ClearITPendingBit(TIM1,TIM_IT_CC3);
  if(flag1==0)
    {
      flag1 = 1;
      t1_one = TIM_GetCounter(TIM2);
    }
    else if(flag1==1)
    {
         if(TIM_GetCounter(TIM2)-t1_one<2100)
       {
        flag1 = 0;
        t1 = (int)(TIM_GetCounter(TIM2)- t1_one -1500)*roll_coeff;

       }
      else
      {

        t1_one = TIM_GetCounter(TIM2);
      }
    }


}

}


void print_ang(int choice,int dt)
{
    char dummy[100];

    if(choice==0)
 
        sprintf(dummy,"ROLL %f PITCH %f YAW %f \r\n",roll,pitch,yaw);

    else if(choice==1)

        sprintf(dummy,"ARM %d THROTTLE %d PITCH %d ROLL %d YAW %d \r\n",arm,t3,t2,t1,t4);

    else

        sprintf(dummy,"%d \r\n",dt);
 
    OutString(dummy);

}




void TIM6_DAC1_IRQHandler(void)
{

      if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);


 int *correct = pid();

  
  uint16_t Channel1Pulse = 0,Channel2Pulse = 0,Channel3Pulse = 0,Channel4Pulse = 0;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

 uint16_t TimerPeriod = (16000000/250) - 1;

  if(arm>1500&&t3>1050)                     //ARMING ONLY DONE IF ARM SWITCH ENABLED AND THROTTLE NOT 1000
  {
  Channel1Pulse = (uint16_t) (((uint32_t) constrain((t3 + t4 - t2 - t1 ),1000,2000)/40 * (TimerPeriod - 1 )) / 100) - 1;
  Channel2Pulse = (uint16_t) (((uint32_t) constrain((t3 - t4 + t2 - t1 ),1000,2000)/40 * (TimerPeriod - 1 )) / 100) - 1;
  Channel3Pulse = (uint16_t) (((uint32_t) constrain((t3 + t4 + t2 + t1 ),1000,2000)/40 * (TimerPeriod - 1 )) / 100) - 1;
  Channel4Pulse = (uint16_t) (((uint32_t) constrain((t3 - t4 - t2 + t1 ),1000,2000)/40 * (TimerPeriod - 1 )) / 100) - 1;
  }
 else
  {
  Channel1Pulse = (uint16_t) ((uint32_t) 25 * (TimerPeriod - 1 ) / 100) - 1;
  Channel2Pulse = (uint16_t) ((uint32_t) 25 * (TimerPeriod - 1 ) / 100) - 1;
  Channel3Pulse = (uint16_t) ((uint32_t) 25 * (TimerPeriod - 1 ) / 100) - 1;
  Channel4Pulse = (uint16_t) ((uint32_t) 25 * (TimerPeriod - 1 ) / 100) - 1;     /
  }
 

 TIM1->CCR4 = Channel1Pulse;
 TIM3->CCR2 = Channel2Pulse;
 TIM3->CCR3 = Channel3Pulse;
 TIM3->CCR4 = Channel4Pulse;
 

    }
}


uint16_t constrain(uint16_t x,uint16_t a,uint16_t b)
{
    if(x<a)
        return a;
    else if(x>b)
        return b;
    else
        return x;
}


