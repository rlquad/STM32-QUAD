
#include "pid_set.h"
#include "mpu6050.h"
#include "mpu_setup.h"
#include "angle.h"

int Kp,Kd = 0;                                                          //PID COEFFICIENTS

int roll_setpoint=0,  pitch_setpoint=0,yaw_setpoint=0;
int roll_last_error,pitch_last_error,yaw_last_error;

int* pid()
{
    int correct[2];                                               //COORECT[0] IS FOR ROLL AND CORRECT[1] IS FOR PITCH
    
    int roll_error  = roll_setpoint  - roll;
    int pitch_error = pitch_setpoint - pitch;
    
    correct[0] =  Kp*roll_error  + Kd*(roll_error  - roll_last_error);
    correct[1] = -Kp*pitch_error - Kd*(pitch_error - pitch_last_error);
    
    roll_last_error  = roll_error;
    pitch_last_error = pitch_error;
    
    return correct;
}
