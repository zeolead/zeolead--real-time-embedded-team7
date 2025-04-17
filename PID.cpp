#include <iostream>
#include <cmath>
#include <unistd.h>
#include <deque>
#include "DEV_Config.hpp"
#include "DRV8825.hpp"
#include "PID.hpp"

extern DRV8825::Motor motor;

float vertical_kp = 1.8, vertical_ki = 0, vertical_kd = 0.1;
float velocity_kp = 1.2, velocity_ki = 0.1, velocity_kd = 0.0;
float turn_kp, turn_ki, turn_kd;

static int Err_lowout_last = 0;
static int intergate_save = 0;
float last_motor_output = 0.0;
float target_velocity = 0.0;

int sensor_dt = 10;
const int max_history = 100/sensor_dt;
std::deque<float> pitch_history;

float PID::Vertical(float angle_output, float pitch, float ax)// 期望角度，真实角度，角速度
{
    // below is the code to estimate pitch accelelation, to join the D part of algorithm
    pitch_history.push_back(pitch);
    if (pitch_history.size() > max_history){
        pitch_history.pop_front();
    }
    float pitch_accle = 0.0;
    if (pitch_history.size() == max_history) {
        float sum_t = 0.0f, sum_tt = 0.0f;
        float sum_y = 0.0f, sum_ty = 0.0f;
        for (int i = 0; i < max_history; ++i) {
            float t = i * sensor_dt * 0.001;
            float y = pitch_history[i];
            sum_t += t;
            sum_tt += t * t;
            sum_y += y;
            sum_ty += t * y;
        }
        float n = static_cast<float>(max_history);
        float denominator = (n * sum_tt - sum_t * sum_t);
        if (denominator != 0.0f) {
            pitch_accle = (n * sum_ty - sum_t * sum_y) / denominator;  // deg/s
            if (pitch_accle < 50 && pitch_accle > -50){
                pitch_accle = 0.0;
            }
        }
    }
    //above is the code to estimate pitch accelelation, to join the D part of algorithm
          
    float balance_output = (vertical_kp * (pitch - angle_output) + vertical_kd * pitch_accle ) + 9;

    if (balance_output > 200) balance_output = 200;
    if (balance_output < -200) balance_output = -200;

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
    float pitch_scale = 0.02f;  // 调大更敏感，调小更稳
    estimated_velocity = 0.8f * estimated_velocity + 0.2f * (pitch_diff * pitch_scale);
    if (std::abs(estimated_velocity) < 0.5f) {
    estimated_velocity = 0.0f;
    }
       
    // slow the update of the velocity

    float error = target_velocity - estimated_velocity;
    // dead zone limt
    if (std::abs(error) <0.2f) {
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
    if(target_velocity != 0.0f && std::abs(estimated_velocity)<0.1f && std::abs(angle_output<1.0f))
        { angle_output=(target_velocity>0) ? 2.0f:-2.0f;
            }
    if (angle_output>20.0f)angle_output=20.0f; 
    if (angle_output<-20.0f)angle_output=-20.0f;        
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
