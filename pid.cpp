#include"pid.h"
float vertical_kp,vertical_ki,vertical_kd;
// pid for stay state//
int Vertical(float Expect,float Angle, float gyro)
{   ;
    int temp;
    temp=vertical_kp*(Angle-Expect)+vertical_kd*gyro;
    return temp;
}
// pid for speed//
float error;
float Velocity( float Expect_velocity,float velocityL,float velocityR )
{
  error=(velocityL+velocityR)-Expect_velocity;
  static int Err_lowout_last,intergate_save;
  float a=0.7;
  int Err,Err_lowout;
  Err_lowout=(1-a)*Err+a*Err_lowout_last;
  Err_lowout_last=Err_lowout;
  // intergate
  intergate_save+=Err_lowout;
    if (intergate_save>20000)
   {
    intergate_save=20000;
   }
   else if (intergate_save<-20000)
   {
    intergate_save=-20000;
   }
   //及时停止
   //if（stop==1){intergate_save=0;stop=0;}
 int temp_v;
 temp_v=vertical_kp*Err_lowout+vertical_ki*intergate_save;
 return temp_v;

}
