#include <iostream>
#include <cmath>
#include <unistd.h> 
#include "DEV_Config.hpp"
#include "DRV8825.hpp"
#include "PID.hpp"

extern DRV8825::Motor motor;

static int Err_lowout_last = 0;
static int intergate_save = 0;
float last_motor_output = 0.0;



float PID::Vertical(float angle_output, float pitch, float ax)// 期望角度，真实角度，角速度
{
    float balance_output = vertical_kp * (pitch - angle_output) + vertical_kd * ax;
    last_motor_output = balance_output;

    if (balance_output > 234) balance_output = 234;
    if (balance_output < -234) balance_output = -234;

    if (outputCallback) {
        outputCallback(balance_output);
    }

    return balance_output;
}

float PID::Velocity()
{
    // === 用 pitch 变化估算系统速度（代替 last_motor_output） ===
    static float estimated_velocity = 0;
    static float last_pitch = 0.0f;
    
    float pitch_diff = current_pitch - last_pitch;
    last_pitch = current_pitch;
    
    // 可调比例因子：pitch_diff 与系统速度关系，需调参
    float pitch_scale = 0.1f;  // 调大更敏感，调小更稳
    estimated_velocity = 0.8f * estimated_velocity + 0.2f * (pitch_diff * pitch_scale);
    // slow the update of the velocity

    float error = target_velocity - estimated_velocity;
    // dead zone limit
    if (std::abs(error) < 0.5f) {
        error = 0;
    }

    float a = 0.7;
    int Err = error;
    int Err_lowout = (1 - a) * Err + a * Err_lowout_last;
    Err_lowout_last = Err_lowout;
    //limit intergrater
    intergate_save += Err_lowout;
    if (intergate_save > 20000)
        intergate_save = 20000;
    if (intergate_save < -20000)
        intergate_save = -20000;

    float angle_output = velocity_kp * Err_lowout + velocity_ki * intergate_save;
    if (target_velocity != 0.0f && std::abs(estimated_velocity) < 0.1f && std::abs(angle_output) < 1.0f) {
        angle_output = (target_velocity > 0) ? 2.0f : -2.0f; 

     //limit angle_output
    if (angle_output > 30.0f) angle_output = 30.0f;
    if (angle_output < -30.0f) angle_output = -30.0f;    
    }
    return angle_output;
}
void PID::setOutputCallback(std::function<void(float)> callback) {
    outputCallback = callback;
}
void PID::receiveSensorData(float pitch, float ax) {
    this->current_pitch = pitch;
    //add quiet conter to test whether it remain stay still
    static int quiet_counter = 0;
    if (std::abs(pitch) < 10.0 && std::abs(last_motor_output) < 10.0) {
        quiet_counter++;
        if (quiet_counter > 300) {  
            intergate_save *= 0.95f;
            if (std::abs(intergate_save) < 100)
                intergate_save = 0;
        }
    } else {
        quiet_counter = 0;  
    }
    float target_angle = Velocity();                    
    float motor_output = Vertical(target_angle, pitch, ax); 
    

    if (outputCallback) {
        outputCallback(motor_output);                   
    }
}

void PID::setParamCallback(std::function<void()> callback) {
    paramCallback = callback;
}

void PID::triggerParamCallback() {
    if (paramCallback) paramCallback();
}

void PID::receivePIDParams(float ver_kp,  float ver_kd ,float vel_kp  ,float vel_ki, float velocity )
{
    std::cout<<"111111"<<std::endl;
    vertical_kp = ver_kp;
    vertical_kd = ver_kd;

    velocity_kp = vel_kp;
    velocity_ki = vel_ki; 
    target_velocity=velocity;
    
}
