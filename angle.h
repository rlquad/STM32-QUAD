
#ifndef setup_h
#define setup_h

#define gyrow       0.996                                                          //WEIGHT OF GYRO FOR COMPLIMENTARY FILTER CALUCULATION
#define accelw      0.004                                                          //WEIGHT OF ACCELEROMETER FOR COMPLIMENTARY FILTER CALCULATION


extern float roll,pitch,yaw;                                                  //DOUBLE OR FLOAT?????????????


void angle_calc(int dt);                                                           //ANGLE CALCULATE


#endif /* setup_h */
