#include <iostream>
#include <cmath>
#include <unistd.h> 
#include "DEV_Config.hpp"
#include "DRV8825.hpp"
#include "PID.hpp"

extern DRV8825::Motor motor;

float vertical_kp = 0.2, vertical_ki = 0.2, vertical_kd = 0.2;
float velocity_kp = 0.1, velocity_ki = 0.1, velocity_kd = 0.1;
float turn_kp, turn_ki, turn_kd;

static int Err_lowout_last = 0;
static int intergate_save = 0;
float last_motor_output = 0.0;


float PID::Vertical(float Expect, float Angle, float gyro_x)
{
    float balance_output = vertical_kp * (Angle - Expect) + vertical_kd * gyro_x;
    return balance_output;
}

float PID::Velocity(float angle_pid_output)
{
    float estimated_velocity = last_motor_output; 
    float error = estimated_velocity - angle_pid_output;

    float a = 0.7;
    int Err = error;
    int Err_lowout = (1 - a) * Err + a * Err_lowout_last;
    Err_lowout_last = Err_lowout;

    intergate_save += Err_lowout;
    if (intergate_save > 20000)
        intergate_save = 20000;
    if (intergate_save < -20000)
        intergate_save = -20000;

    float motor_output = velocity_kp * Err_lowout + velocity_ki * intergate_save;
    last_motor_output = motor_output;
    if (outputCallback) {
        outputCallback(motor_output);}
    return motor_output;
}

void PID::setOutputCallback(std::function<void(float)> callback) {
    outputCallback = callback;
}
void PID::receiveSensorData(float pitch, float ax, float gx) {
    float vertical_output = Vertical(0.0f, pitch, gx);  
    float velocity_output = Velocity(vertical_output);
    if (outputCallback) {
      outputCallback(velocity_output);
    }
}
