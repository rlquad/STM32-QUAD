#include "stm32.h"


uint32_t timer1;


void setup()
{
 init();  
 imu_config();
 calib();
 angle_calc_init();

 timer1 = TIM_GetCounter(TIM2);
}


int main()
{ 
  int dt;
 
  setup();
  
  while(1)
  {    
 
   dt = TIM_GetCounter(TIM2) - timer1;
   angle_calc(dt);
   timer1 = TIM_GetCounter(TIM2);

 //print_ang(0,0);

  }
  return 0;
}
