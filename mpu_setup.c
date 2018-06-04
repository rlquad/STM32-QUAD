#include "mpu_setup.h"
#include "mpu6050.h"
#include "angle.h"
#include "stdio.h"
#include <math.h>



///////////////////////////////////////////////////////////////////////////////////////////////
//TIMER INITIALISATION USED TO ENABLE:
//PWM GENERATION
//GLOBAL CLOCK (ANGLE CALCULATION etc)
//5 CHANNEL RECEIVER INPUT
///////////////////////////////////////////////////////////////////////////////////////////////


void tim_init()
{
///////////////////////////////////////////////////////////////////////////////////////////////
//PWM GENERATION USED TO CONTROL MOTORS. IT IS DONE THROUGH TIMERS 3 AND 1
//    MOTOR   TIMER   CHANNEL   PIN   PORT
//    1       TIM1    4         D10   PA11
//    2       TIM3    2         A6    PA7
//    3       TIM3    3         D3    PB0
//    4       TIM3    4         D6    PB1
///////////////////////////////////////////////////////////////////////////////////////////////

  uint16_t Channel1Pulse = 0,Channel2Pulse = 0,Channel3Pulse = 0,Channel4Pulse = 0;        //DEFINITION OF DUTY CYCLE OF GENERATED PWM'S

  TIM_OCInitTypeDef  TIM_OCInitStructure;

  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef SetupTimer;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //ALTERNATE FUNCTION INITIALISATION

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_11);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);

   uint16_t TimerPeriod = (16000000/250) - 1;                                      //TIMER PERIOD DEFINITON - DONT KNOW WHY????

   //TimerPeriod = (8000000/250) - 1;

  Channel1Pulse = (uint16_t) (((uint32_t) 25 * (TimerPeriod - 1 )) / 100) - 1;     //DUTY CYCLE OF 25 USED FOR ARMING ESC
  Channel2Pulse = (uint16_t) (((uint32_t) 25 * (TimerPeriod - 1 )) / 100) - 1;     //DUTY CYCLE OF 25 USED FOR ARMING ESC
  Channel3Pulse = (uint16_t) (((uint32_t) 25 * (TimerPeriod - 1 )) / 100) - 1;     //DUTY CYCLE OF 25 USED FOR ARMING ESC
  Channel4Pulse = (uint16_t) (((uint32_t) 25 * (TimerPeriod - 1 )) / 100) - 1;     //DUTY CYCLE OF 25 USED FOR ARMING ESC

  //CLOCK ENABLING FOR TIMER 1 AND 3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);

  SetupTimer.TIM_Prescaler = 3;                                                    //DONT KNOW WHY
  SetupTimer.TIM_CounterMode = TIM_CounterMode_Up;
  SetupTimer.TIM_Period = TimerPeriod;
  SetupTimer.TIM_ClockDivision = 0;
  SetupTimer.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &SetupTimer);
  TIM_TimeBaseInit(TIM3, &SetupTimer);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
  TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
  TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM1, ENABLE);

  TIM_CtrlPWMOutputs(TIM1, ENABLE);


  //for(float i=0;i<5000000;i++);                                                //WAIT FOR SOME TIME TO GIVE ARM SEQUENCE TO ESC

//////////////////////////////////////////////////////////////////////////////////////////////
//TIMER 2 - GLOBAL CLOCK
//5 CHANNEL RECEIVER INITIALISATION
// RECEIVER-CHANNEL    TIMER     TIMER-CHANNEL    PIN
// 1                   2         1                A0
// 2                   2         4                A2
// 3                   1         1                D9
// 4                   1         3                D0
// 5                   2         2                A1
//////////////////////////////////////////////////////////////////////////////////////////////


  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  SetupTimer.TIM_Prescaler = 64 - 1;                                                    // Prescale value is 63

  SetupTimer.TIM_CounterMode = TIM_CounterMode_Up;
  SetupTimer.TIM_Period = 0xFFFFFFFF;                                                  //RESTART THE TIMER AFTER THIS VALUE
  SetupTimer.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &SetupTimer);

  GPIO_InitTypeDef gpio_init;

  gpio_init.GPIO_Pin  = GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_0|GPIO_Pin_8|GPIO_Pin_10;
  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;

  GPIO_Init(GPIOA, &gpio_init);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_6);

  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ICInitTypeDef in;

  in.TIM_Channel = TIM_Channel_2;
  in.TIM_ICFilter = 0x00;
  in.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  in.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  in.TIM_ICSelection = TIM_ICSelection_DirectTI;

  TIM_ICInit(TIM2,&in);

  in.TIM_Channel = TIM_Channel_4;
  TIM_ICInit(TIM2,&in);

  in.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM2,&in);

  in.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM1,&in);

  in.TIM_Channel = TIM_Channel_3;
  TIM_ICInit(TIM1,&in);

  TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);
  TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
  TIM_ITConfig(TIM2,TIM_IT_CC2,ENABLE);
  TIM_ITConfig(TIM1,TIM_IT_CC3,ENABLE);
  TIM_ITConfig(TIM2,TIM_IT_CC4,ENABLE);

  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM1, ENABLE);

///////////////////////////////////////////////////////////////////////////////////////////
//TIMER 6 FOR SENDING MOTOR SIGNALS EVERY 4000us. INTERRUPT DEFINITION
//////////////////////////////////////////////////////////////////////////////////////////
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);


  SetupTimer.TIM_Prescaler = 64 - 1;                               // Prescale value is 63

  SetupTimer.TIM_CounterMode = TIM_CounterMode_Up;
  SetupTimer.TIM_Period = 4458;
  //SetupTimer.TIM_Period = 3999 + 575;                                    //DONT KNOW WHY THIS PERIOD (FOUND THROUGH TRIAL AND ERROR)
  SetupTimer.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM6, &SetupTimer);

 TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM6, ENABLE);
//////////////////////////////////////////////////////////////////////////////////////////
}


void uart_init()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);

  GPIO_InitTypeDef gpio_init;

  gpio_init.GPIO_Pin = GPIO_Pin_2;
  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  gpio_init.GPIO_Speed = GPIO_Speed_Level_3;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOA, &gpio_init);

  USART_InitTypeDef usart1;

  usart1.USART_BaudRate =  115200;
  usart1.USART_WordLength = USART_WordLength_8b;
  usart1.USART_StopBits = USART_StopBits_1;
  usart1.USART_Parity = USART_Parity_No;
  usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart1.USART_Mode = USART_Mode_Tx;

  USART_Init(USART2, &usart1);

  USART_Cmd(USART2, ENABLE);

}

void OutString(char *s)
{
  while(*s)
  {
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);                     // Wait for Empty

    USART_SendData(USART2, *s++);                                                   // Send Char
  }
}

void init()
{

  FLASH_PrefetchBufferCmd(ENABLE);
  FLASH_SetLatency(FLASH_Latency_2);                                                //FLASH LATENCY HAS TO BE CHANGED FROM 0 TO 2

  RCC_PLLConfig(RCC_CFGR_PLLXTPRE_PREDIV1,RCC_PLLMul_16);

  RCC_PLLCmd(ENABLE);
  while(RCC_CR_PLLRDY==0);

  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  RCC_PCLK1Config(RCC_HCLK_Div2);
  RCC_PCLK2Config(RCC_HCLK_Div1);

  RCC->CFGR3 |= (uint32_t)0x00000010;                                               //SYSCLK ASSIGNED TO I2C1. I2C1 DEFAULT IS HSI

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);

      tim_init();
      uart_init();
      pwm_init();

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);

  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_4);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_4);

  GPIO_InitTypeDef gpio_configure;

  gpio_configure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
  gpio_configure.GPIO_Mode =  GPIO_Mode_AF;
  gpio_configure.GPIO_Speed = GPIO_Speed_Level_1;
  gpio_configure.GPIO_OType = GPIO_OType_OD;
  gpio_configure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOB,&gpio_configure);

  I2C_InitTypeDef i2c_config;

  //i2c_config.I2C_Timing              = 0x00C0216C;                                  //FOR 64MHZ - 400kHz
  i2c_config.I2C_Timing              = 0x0010020A ;                               //FOR 8MHz  - 400kHz
  //i2c_config.I2C_Timing              = 0x00201D2B;                                //FOR 8MHz  - 100kHz
  i2c_config.I2C_AnalogFilter        = I2C_AnalogFilter_Enable;
  i2c_config.I2C_DigitalFilter       = 0x00;
  i2c_config.I2C_Mode                = I2C_Mode_I2C;
  i2c_config.I2C_OwnAddress1         = 0x00;
  i2c_config.I2C_Ack                 = I2C_Ack_Enable;                              //CHECK OUT WHY ACK IS ENABLED
  i2c_config.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

  I2C1->CR1 &= ~I2C_CR1_PE;
  I2C_Init(I2C1,&i2c_config);

  I2C_SlaveAddressConfig(I2C1, MPU_ADD);                                             //MPU6050 ADDRESS IS 0X68

  I2C_Cmd(I2C1, ENABLE);

 // imu_config();
 // int c = check_imu_connect();

/*  if(c==0x68)
  {

      tim_init();
      uart_init();
      pwm_init();
  }
  else

  {
       LED_state(0);
  }*/
}


int check_imu_connect()
{
    wrt_reg(WHO_AM_I,1);
    return recv_byte();

}


void wrt_reg(int reg,int bytes)
{
  I2C_MasterRequestConfig(I2C1, I2C_Direction_Transmitter);                         //DEFINING DIRECTION OF DATA TRANSMISSION

  I2C_NumberOfBytesConfig(I2C1, bytes);                                             //NUMBER OF BYTES TO BE TRANSMITTED

  I2C_GenerateSTART(I2C1,ENABLE);                                                   //GENERATE START CONDITION
  while(I2C1->CR2 & I2C_CR2_START);                                                 //TO ENSURE START CONDITION HAS BEEN FULFILLED

  I2C_SendData(I2C1, reg);                                                         //WRITE DATA TO A SPECIFIC REGISTER
  while(!(I2C1->ISR & I2C_ISR_TXE));                                                //TO ENSURE DATA HAS BEEN WRITTEN TO TXE BIT

}

void wrt_reg_data(int reg,int data,int bytes)
{
  I2C_MasterRequestConfig(I2C1, I2C_Direction_Transmitter);                         //DEFINING DIRECTION OF DATA TRANSMISSION

  I2C_NumberOfBytesConfig(I2C1, bytes);                                             //NUMBER OF BYTES TO BE TRANSMITTED

  I2C_GenerateSTART(I2C1,ENABLE);                                                   //GENERATE START CONDITION
  while(I2C1->CR2 & I2C_CR2_START);                                                 //TO ENSURE START CONDITION HAS BEEN FULFILLED


  I2C_SendData(I2C1, reg);                                                          //WRITE DATA TO A SPECIFIC REGISTER
  while(!(I2C1->ISR & I2C_ISR_TXE));                                                //TO ENSURE DATA HAS BEEN WRITTEN TO TXE BIT

  I2C_SendData(I2C1, data);                                                          //WRITE DATA TO A SPECIFIC REGISTER
  while(!(I2C1->ISR & I2C_ISR_TXE));                                                //TO ENSURE DATA HAS BEEN WRITTEN TO TXE BIT
}

int recv_byte()
{
  I2C_MasterRequestConfig(I2C1, I2C_Direction_Receiver);                            //DEFINING DIRECTION OF DATA TRANSMISSION

  I2C_NumberOfBytesConfig(I2C1, 1);                                               //NUMBER OF BYTES TO RECEIVE FROM SLAVE

  I2C_GenerateSTART(I2C1,ENABLE);                                                    //GENERATE START CONDITION
  while(I2C1->CR2 & I2C_CR2_START);                                                 //TO ENSURE START CONDITION HAS BEEN FULFILLED

  while(!(I2C1->ISR & I2C_ISR_RXNE));                                               //CHECKING IF DATA IS RECEIVED
  int data = I2C_ReadRegister(I2C1,I2C_Register_RXDR);

  I2C_GenerateSTOP(I2C1,ENABLE);                                                    //GENERATE STOPPING CONDITION
  while(I2C1->CR2 & I2C_CR2_STOP);                                                  //ENSURE STOP CONDITION IS ENFORCED BEFORE PROCEEDING

return data;

}


void recv_orient_data()
{                                                                                   //TO ENABLE ACKNOWLEDGEMENT BY THE MASTER AFTER RECEIVING THE DATA

  wrt_reg(obt_orient,1);

  I2C_MasterRequestConfig(I2C1, I2C_Direction_Receiver);                            //DEFINING DIRECTION OF DATA TRANSMISSION

  I2C_NumberOfBytesConfig(I2C1, 14);                                                //NUMBER OF BYTES TO RECEIVE FROM SLAVE

  I2C_GenerateSTART(I2C1,ENABLE);                                                   //GENERATE START CONDITION
  while(I2C1->CR2 & I2C_CR2_START);                                                 //TO ENSURE START CONDITION HAS BEEN FULFILLED

  while(!(I2C1->ISR & I2C_ISR_RXNE));                                               //CHECKING IF DATA IS RECEIVED
  acc[0] = I2C_ReadRegister(I2C1,I2C_Register_RXDR);
  while(!(I2C1->ISR & I2C_ISR_RXNE));
  acc[0] = (acc[0]<<8)|I2C_ReadRegister(I2C1,I2C_Register_RXDR);
  acc[0] = (acc[0]<<acc_smt)>>acc_smt;


  while(!(I2C1->ISR & I2C_ISR_RXNE));                                               //CHECKING IF DATA IS RECEIVED
  acc[1] = I2C_ReadRegister(I2C1,I2C_Register_RXDR);
  while(!(I2C1->ISR & I2C_ISR_RXNE));
  acc[1] = (acc[1]<<8)|I2C_ReadRegister(I2C1,I2C_Register_RXDR);
  acc[1] = (acc[1]<<acc_smt)>>acc_smt;

  while(!(I2C1->ISR & I2C_ISR_RXNE));                                               //CHECKING IF DATA IS RECEIVED
  acc[2] = I2C_ReadRegister(I2C1,I2C_Register_RXDR);
  while(!(I2C1->ISR & I2C_ISR_RXNE));
  acc[2] = (acc[2]<<8)|I2C_ReadRegister(I2C1,I2C_Register_RXDR);
  acc[2] = (acc[2]<<acc_smt)>>acc_smt;

  while(!(I2C1->ISR & I2C_ISR_RXNE));                                               //CHECKING IF DATA IS RECEIVED
  I2C_ReadRegister(I2C1,I2C_Register_RXDR);
  while(!(I2C1->ISR & I2C_ISR_RXNE));
  I2C_ReadRegister(I2C1,I2C_Register_RXDR);

  while(!(I2C1->ISR & I2C_ISR_RXNE));                                               //CHECKING IF DATA IS RECEIVED
  gyro[0] = I2C_ReadRegister(I2C1,I2C_Register_RXDR);
  while(!(I2C1->ISR & I2C_ISR_RXNE));
  gyro[0] = (gyro[0]<<8)|I2C_ReadRegister(I2C1,I2C_Register_RXDR);


  while(!(I2C1->ISR & I2C_ISR_RXNE));                                               //CHECKING IF DATA IS RECEIVED
  gyro[1] = I2C_ReadRegister(I2C1,I2C_Register_RXDR);
  while(!(I2C1->ISR & I2C_ISR_RXNE));
  gyro[1] = (gyro[1]<<8)|I2C_ReadRegister(I2C1,I2C_Register_RXDR);


  while(!(I2C1->ISR & I2C_ISR_RXNE));                                               //CHECKING IF DATA IS RECEIVED
  gyro[2] = I2C_ReadRegister(I2C1,I2C_Register_RXDR);
  while(!(I2C1->ISR & I2C_ISR_RXNE));
  gyro[2] = (gyro[2]<<8)|I2C_ReadRegister(I2C1,I2C_Register_RXDR);


  I2C_GenerateSTOP(I2C1,ENABLE);                                                    //GENERATE STOPPING CONDITION
  while(I2C1->CR2 & I2C_CR2_STOP);                                                  //ENSURE STOP CONDITION IS ENFORCED BEFORE PROCEEDING

}

void imu_config()
{

  wrt_reg_data(PWR_MGMT,0,2);
  I2C_GenerateSTOP(I2C1,ENABLE);                                                    //GENERATE STOPPING CONDITION
  while(I2C1->CR2 & I2C_CR2_STOP);

  wrt_reg_data(gyro_config,0x18,2);
  I2C_GenerateSTOP(I2C1,ENABLE);                                                    //GENERATE STOPPING CONDITION
  while(I2C1->CR2 & I2C_CR2_STOP);

  wrt_reg_data(acc_config,0x00,2);
  I2C_GenerateSTOP(I2C1,ENABLE);                                                    //GENERATE STOPPING CONDITION
  while(I2C1->CR2 & I2C_CR2_STOP);

  wrt_reg_data(dlpf_reg,0x00,2);
  I2C_GenerateSTOP(I2C1,ENABLE);                                                    //GENERATE STOPPING CONDITION
  while(I2C1->CR2 & I2C_CR2_STOP);

}

void angle_calc_init()
{
  recv_orient_data();

  roll  = atan2(acc[1],acc[2]);
  pitch = atan2(-acc[0],acc[2]);
  yaw   = 0;

    dcm[0][0] = cos(pitch);
    dcm[0][1] = sin(roll)*sin(pitch);
    dcm[0][2] = cos(roll)*sin(pitch);
    dcm[1][0] = 0;
    dcm[1][1] = cos(roll);
    dcm[1][2] = -sin(roll);
    dcm[2][0] = -sin(pitch);
    dcm[2][1] = sin(roll)*cos(pitch);
    dcm[2][2] = cos(roll)*cos(pitch);
}

//////////////////////////////////////////////////////////////////////////////////////
//FOR CALLING MOTOR CONTROL CODE EVERY 4000us
//////////////////////////////////////////////////////////////////////////////////////

void pwm_init()
{

  NVIC_InitTypeDef  NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}
