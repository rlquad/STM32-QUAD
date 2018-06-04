
#ifndef MPU_SETUP_H
#define MPU_SETUP_H
#include "stm32f30x.h"

#define dlpf_reg    0x1A                                                           //DIGITAL LOW PASS FILTER
#define gyro_config 0x1B
#define acc_config  0x1C
#define WHO_AM_I    0x75                                                           //ADDRESS OF MPU6050 IS STORED IN THIS REGISTER
#define PWR_MGMT    0x6B
#define MPU_ADD     0x68                                                           //ADDRESS OF MPU


#define acc_sens    0x0
#define gyro_sens   0x3

#define acc_smt     8



void tim_init();                                                                   //TIMER INITIALISATION
void uart_init();                                                                  //USART INITIALISATION
void OutString(char *s);                                                           //TO PRINT OUT
void init();                                                                       //I2C INITIALISATION
void wrt_reg(int reg,int bytes);                                                   //BROADCAST REGISTER ON THE I2C LINE
void wrt_reg_data(int reg,int data,int bytes);                                     //BRODCAST AND WRITE TO A SPECIFIC REGISTER
int recv_byte();                                                                   //RECEIVE A SINGLE BYTE FROM THE REGISTER WRITTEN TO
void recv_orient_data();                                                           //OBTAIN THE ACCELEROMETER AND GYROSCOPE DATA FROM MPU
void imu_config();                                                                 //CONFIGURE THE IMU
void angle_calc_init();                                                            //TO CALCULATE FIRST ANGLE AT STABLE STATE
void pwm_init();                                                                   //TO CALL MOTOR CONTROL CODE EVERY 4000us

#endif
