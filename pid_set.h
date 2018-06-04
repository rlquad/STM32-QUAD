
#ifndef pid_set_h
#define pid_set_h

 extern int Kp,Kd;
extern int roll_setpoint, pitch_setpoint, yaw_setpoint;
extern int roll_last_error,pitch_last_error,yaw_last_error;


int* pid();                                                 //PID CORRECTION BASED ON RECEIVER INPUT AND QUAD ORIENTATION

#endif
