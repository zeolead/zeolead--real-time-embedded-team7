#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include "mpu6050_test001.hpp"
#include "PID.hpp"
#include "MotorControl.hpp"

void Handler(int sig) {
    std::cout << "\nHandler: Manual Stop" << std::endl;
    DEV_Config::DEV_ModuleExit();
    exit(0);
}


int main() {

    // initialize
    MPU6050 mpu;
    PID pid;
    if (DEV_Config::DEV_ModuleInit()) {
        return 0;
    }
    std::signal(SIGINT,Handler);
    MotorControl motor1(DRV8825::MOTOR1);
    MotorControl motor2(DRV8825::MOTOR2);
    

    mpu.setCallback([&](float pitch, float ax) {
        pid.receiveSensorData(pitch, ax); 
    });
    
    pid.setOutputCallback([&](float output) {
        int rpm = static_cast<int>(output);
        motor1.setRPM(rpm);
        motor2.setRPM(-rpm);
        
    });
    
    motor1.start();
    motor2.start();
    
    std::thread mpuThread(&MPU6050::run, &mpu);
    mpuThread.join(); 

    std::cout << "close system" << std::endl;

    return 0;
}
