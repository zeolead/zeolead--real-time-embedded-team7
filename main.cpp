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


int main(int argc, char* argv[]) {
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
    
    motor1.start(1);
    motor2.start(1);

    //motor1.TurnStep(DRV8825::FORWARD, 10, 50);   //10 pluses for 4.5 degree
    //motor2.TurnStep(DRV8825::BACKWARD, 10, 50);
    
    std::thread mpuThread(&MPU6050::run, &mpu);
    mpuThread.join(); 

    std::cout << "close system" << std::endl;

    return 0;
}
