
#ifndef MPU6050_H
#define MPU6050_H
#include "stm32f30x.h"

#define obt_orient  0x3B                                                           //ADDRESS FOR GETTING COMPLETE ORIENTATION DATA

#define roll_coeff  0.25
#define pitch_coeff 0.25
#define yaw_coeff   0.25

#define acc_lsb     16384                                                          //CONVERSION FACTOR FOR ACCELERATION OUTPUT
#define gyro_lsb    16.4                                                           //CONVERSION FACTOR FOR GYROSCOPE OUTPUT
#define degconvert  57.2957795

 extern int16_t gyro[3],acc[3];

 extern uint32_t t1_one,t2_one,t3_one,t4_one,arm_one;
 extern int t1,t2,t3,t4,arm;
 extern int flag1,flag2,flag3,flag4,flag5;
 extern float dcm[3][3];

 extern uint32_t tim2;
void print_ang(int ,int);                                                            //CALL OutString TO PRINT ANGLE DATA TO USART
void EXTI1_IRQHandler(void);                                                       //EXTERNAL INTERRUPT HANDLER

void TIM3_IRQHandler(void);                                                         //CHANGE DUTY CYCLE OF PWM OUTPUT

uint16_t constrain(uint16_t,uint16_t,uint16_t);
void TIM2_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void LED_state(int);

#endif
