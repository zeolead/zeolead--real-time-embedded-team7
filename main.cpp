#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include "mpu6050_Kalman.hpp"
#include "PID.hpp"
#include "MotorControl.hpp"

void Handler(int sig) {
    std::cout << "\nHandler: Manual Stop" << std::endl;
    DEV_Config::DEV_ModuleExit();
    exit(0);
}
void updatePIDParams(PID& pid) {
    pid.receivePIDParams(
        0.3f, 0.1f,   // vertical_kp, vertical_kd
        0.2f, 0.05f,  // velocity_kp, velocity_ki
        0.0f          // target_velocity
    );
}

int main() {
    int turn1;
    int turn2;
    // initialize
    MPU6050 mpu;
    PID pid;
    if (DEV_Config::DEV_ModuleInit()) {
        return 0;
    }
    std::signal(SIGINT,Handler);
    MotorControl motor1(DRV8825::MOTOR1);
    MotorControl motor2(DRV8825::MOTOR2);
    
    pid.setParamCallback(std::bind(updatePIDParams, std::ref(pid)));
    pid.triggerParamCallback();

    mpu.setCallback([&](float pitch, float ax) {
        pid.receiveSensorData(pitch, ax); 
    });
    
    pid.setOutputCallback([&](float output) {
        int rpm = std::static_cast<int>(output);
        motor1.setRPM(rpm+turn1);
        motor2.setRPM(-rpm+turn2);
        
    });
    
    motor1.start();
    motor2.start();
    
    std::thread mpuThread(&MPU6050::run, &mpu);
    mpuThread.join(); 

    std::cout << "close system" << std::endl;

    return 0;
}
