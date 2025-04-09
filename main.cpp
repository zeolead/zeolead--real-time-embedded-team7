#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>
#include <unistd.h>
#include "mpu6050_test001.hpp"
#include "PID.hpp"
#include "MotorControl.hpp"




int main() {

    // initialize
    MPU6050 mpu;
    PID pid;
    MotorControl motor1(DRV8825::MOTOR1);
    MotorControl motor2(DRV8825::MOTOR2);
    

    mpu.setCallback([&](float pitch, float ax, float gx) {
        pid.receiveSensorData(pitch, ax, gx); 
        std::cout << "recieve data" << std::endl;
    });
    
    pid.setOutputCallback([&](float output) {
        int rpm = std::abs(static_cast<int>(output));
    
        motor1.setRPM(rpm);
        motor2.setRPM(rpm);
        
    });
    
    motor1.start();
    motor2.start();
    
    std::thread mpuThread(&MPU6050::run, &mpu);
    mpuThread.join(); 

    std::cout << "close system" << std::endl;

    return 0;
}
