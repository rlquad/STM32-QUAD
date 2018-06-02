

#include "setup.h"
#include "mpu6050.h"
#include "mpu_setup.h"

float roll,pitch,yaw;


void angle_calc(int dt)
{
    recv_orient_data();
    
    roll  = accelw*(atan2(acc[1],acc[2])*degconvert)  + gyrow*(roll  + gyro[0]*dt/(1000000.0*gyro_lsb));
    pitch = accelw*(atan2(-acc[0],acc[2])*degconvert) + gyrow*(pitch + gyro[1]*dt/(1000000.0*gyro_lsb));
    yaw   = yaw + gyro[2]*dt/(1000000.0*gyro_lsb);
    
    
}
