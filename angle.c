

#include "angle.h"
#include "mpu6050.h"
#include "mpu_setup.h"
#include <math.h>


float roll,pitch,yaw;
float dtheta[3] = {0, 0, 0};

float dcm[3][3];

float gyrow = 0.996;
float accelw = 0.004;

int16_t gyro_cal[3] = {0, 0, 0};
int16_t acc_cal[3] = {0, 0 ,0};


void angle_calc(int dt)
{
    recv_orient_data();

     for(int i=0;i<3;i++)
  {
      gyro[i] -= gyro_cal[i];
      acc[i]  += acc_cal[i];
  }

    float norm = (acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2])/(16384.0*16384.0);

/*    if(norm>1.3225||norm<0.7225)
    {
        accelw = 0;
        gyrow  = 1;
    }
    else
    {
        accelw = 0.004;
        gyrow  = 0.996;
    }
*/
    dtheta[0] = gyrow*gyro[0]*dt/(1000000.0*gyro_lsb*degconvert) + accelw*(atan2(acc[1],acc[2])-roll);
    dtheta[1] = gyrow*gyro[1]*dt/(1000000.0*gyro_lsb*degconvert) + accelw*(atan2(-acc[0],acc[2])-pitch);
    dtheta[2] = gyro[2]*dt/(1000000.0*gyro_lsb*degconvert);

    vec_rotate();

    roll  = atan2(dcm[2][1],dcm[2][2]);
    pitch = asin(-dcm[2][0]);
    yaw   = atan2(dcm[1][0],dcm[0][0]);


    /*
    roll  = accelw*(atan2(acc[1],acc[2])*degconvert)  + gyrow*(roll  + gyro[0]*dt/(1000000.0*gyro_lsb));
    pitch = accelw*(atan2(-acc[0],acc[2])*degconvert) + gyrow*(pitch + gyro[1]*dt/(1000000.0*gyro_lsb));
    yaw   = yaw + gyro[2]*dt/(1000000.0*gyro_lsb);
    */


}

void vec_rotate()
{
    float temp_dcm[3][3];
    float dot = 0;

    float euler[][3] = {   1          , -dtheta[2] , dtheta[1],
                           dtheta[2]  , 1          , -dtheta[0],
                           -dtheta[1] , dtheta[0]  , 1             };


    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            temp_dcm[i][j] = 0;

            for(int k=0;k<3;k++)
                temp_dcm[i][j] += dcm[i][k]*euler[k][j];


        }
    }

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)
        {dcm[i][j] = temp_dcm[i][j];

        }
    }
    /*
    for(int i=0;i<3;i++)
        dot += dcm[0][i]*dcm[1][i];

    float dummy[2];
    for(int i=0;i<3;i++)
    {
        dummy[0] = dcm[0][i] - dot*dcm[1][i]/2;
        dummy[1] = dcm[1][i] - dot*dcm[0][i]/2;

        dcm[0][i] = dummy[0];
        dcm[1][i] = dummy[1];

    }

    dcm[2][0] = dcm[0][1]*dcm[1][2] - dcm[1][1]*dcm[0][2];
    dcm[2][1] = dcm[1][0]*dcm[0][2] - dcm[0][0]*dcm[1][2];
    dcm[2][2] = dcm[0][0]*dcm[1][1] - dcm[1][0]*dcm[0][1];

     renorm();

     */

}



void renorm()
{
    float rnorm = dcm[0][0]*dcm[0][0] + dcm[0][1]*dcm[0][1] + dcm[0][2]*dcm[0][2];

    for(int i=0;i<3;i++)
        dcm[0][i] = .5*(3 - rnorm)*dcm[0][i];

    rnorm = dcm[1][0]*dcm[1][0] + dcm[1][1]*dcm[1][1] + dcm[1][2]*dcm[1][2];

    for(int i=0;i<3;i++)
        dcm[1][i] = .5*(3 - rnorm)*dcm[1][i];

    rnorm = dcm[2][0]*dcm[2][0] + dcm[2][1]*dcm[2][1] + dcm[2][2]*dcm[2][2];

    for(int i=0;i<3;i++)
        dcm[2][i] = .5*(3 - rnorm)*dcm[2][i];

}


void calib()
{
   for(int i=0;i<500;i++)
    {
        recv_orient_data();
        for(int j=0;j<3;j++)
        {gyro_cal[i] += gyro[i];

        }
         acc_cal[2] += (16384 - acc[2]);
         acc_cal[0] += -acc[0];
         acc_cal[1] += -acc[1];
    }

    for(int i = 0;i<3;i++)
    {
        gyro_cal[i]/=500;
        acc_cal[i]/=500;

    }


}

