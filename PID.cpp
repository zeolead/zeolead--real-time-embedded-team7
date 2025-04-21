#include <iostream>
#include <cmath>
#include <unistd.h> 
#include <deque>
#include "DEV_Config.hpp"
#include "DRV8825.hpp"
#include "PID.hpp"

extern DRV8825::Motor motor;

static int Err_lowout_last = 0;
static int intergate_save = 0;
float last_motor_output = 0.0;
int sensor_dt = 5;
const int max_history = 100 / sensor_dt;
std::deque<float> pitch_history;
constexpr float wheel_radius = 4;


float PID::Vertical(float angle_output, float pitch, float ax)// tatget pitch，current pitch，pitch speed
{
    // below is the code to estimate pitch rotation speed, to join the D part of algorithm
    pitch_history.push_back(pitch);
    if (pitch_history.size() > max_history) {
        pitch_history.pop_front();
    }
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
            pitch_angvol = (n * sum_ty - sum_t * sum_y) / denominator;  // deg/s
            if (pitch_angvol < 1 && pitch_angvol > -1) {
                pitch_angvol = 0.0;
            }
        }
    }
    //above is the code to estimate pitch rotation speed, to join the D part of algorithm
    float balance_output = vertical_kp * (pitch - angle_output) + vertical_kd * pitch_angvol;
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
    //static float last_pitch = 0.0f;
    //
    //float pitch_diff = current_pitch - last_pitch;
    //last_pitch = current_pitch;
    //
    //// 可调比例因子：pitch_diff 与系统速度关系，需调参
    //float pitch_scale = 0.1f;  // 调大更敏感，调小更稳
    //estimated_velocity = 0.8f * estimated_velocity + 0.2f * (pitch_diff * pitch_scale);
    // slow the update of the velocity

    estimated_velocity = wheel_radius * (2 * M_PI * RPM / 60 + 2 * M_PI * pitch_angvol / 360);
    std::cout << "estimated_velocity: " << estimated_velocity << std::endl;
    float error = target_velocity - estimated_velocity;
    // dead zone limit
    if (std::abs(error) < 0.1f) {
        error = 0;
    }

    float a = 0.975;
    int Err = error;
    int Err_lowout = (1 - a) * Err + a * Err_lowout_last;
    Err_lowout_last = Err_lowout;
    //limit intergrater
    intergate_save +=  Err_lowout;
    if (intergate_save > 10000)
        intergate_save = 10000;
    if (intergate_save < -10000)
        intergate_save = -10000;

    float angle_output = velocity_kp * Err_lowout + velocity_ki * intergate_save;
    //std::cout << "Err_lowout: " << Err_lowout << std::endl;
    //std::cout << "intergate_save: " << intergate_save << std::endl;
    //std::cout << "target angle" << angle_output << std::endl;
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

void PID::receiveRPM(float rpm) {
    RPM = rpm;
}