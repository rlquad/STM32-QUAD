
#ifndef angle_h
#define angle_h
#include "stm32f30x.h"

extern float gyrow;                                                          //WEIGHT OF GYRO FOR COMPLIMENTARY FILTER CALUCULATION
extern float accelw ;                                                          //WEIGHT OF ACCELEROMETER FOR COMPLIMENTARY FILTER CALCULATION

extern int16_t gyro_cal[3];
extern int16_t acc_cal[3];

extern float roll,pitch,yaw;                                                  //DOUBLE OR FLOAT?????????????
extern float dtheta[3];

void angle_calc(int dt);                                                           //CALCULATE ANGLE
void renorm();                                                                     //RENORMALISE THE DCM MATRIX
void vec_rotate();                                                                 //ROTATE VECTOR FROM BODY TO GLOBAL FRAME
void calib();
#endif
