#include <iostream>
#include <cmath>
#include <unistd.h> 
#include "DEV_Config.hpp"
#include "DRV8825.hpp"

extern DRV8825::Motor motor;

float vertical_kp=0.2,vertical_ki=0.2,vertical_kd=0.2;
float velocity_kp=0.1,velocity_ki=0.1,velocity_kd=0.1;
float turn_kp,turn_ki,turn_kd;

static int Err_lowout_last = 0;
static int intergate_save = 0;
float last_motor_output = 0.0;
// pid for stay state//
float Vertical(float Expect=0, float Angle, float gyro_x)
{


    float balance_output = vertical_kp * (Angle - Expect) + vertical_kd * gyro_x;
    return balance_output;
}
// pid for speed//
float Velocity( float angle_pid_output)
  {
      float estimated_velocity = last_motor_output; 
      float error = estimated_velocity - angle_pid_output;
  
      float a = 0.7;
      int Err = error;
      int Err_lowout = (1 - a) * Err + a * Err_lowout_last;
      Err_lowout_last = Err_lowout;
  
      intergate_save += Err_lowout;
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
 float temp_v;
 temp_v=velocity_kp*Err_lowout+velocity_ki*intergate_save;
 last_motor_output = temp_v;
    return temp_v;

}
//pid for turn
/*int Turn(float gyro_z,int Target)
{
  int temp;
  temp=turn_kp*Target+turn_kd*gyro_z;
  return temp;
}*/
void driveMotorWithVelocityOutput(float motor_speed_output)
{
    int dir = motor_speed_output > 0 ? 1 : 0;
    float abs_speed = fabs(motor_speed_output);

    if (abs_speed < 1) abs_speed = 1;

    int step_delay_us = static_cast<int>(1e6 / abs_speed);
    int steps = 1;

    for (int i = 0; i < steps; ++i)
    {
      DEV_Config::DEV_Digital_Write(motor.StepPin, 1);  // STEP 高
      DEV_Config::DEV_Digital_Write(motor.StepPin, 0);  // STEP 低

      DEV_Config::DEV_Delay_us(step_delay_us);  // 控制速度
      usleep(step_delay_us);
    }
}