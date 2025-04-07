#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include<gpiod.h>
#include<mutex>
#include"mpu6050_test001.h"
#include"PID.h"
#include "DRV8825.hpp"

// protect data
std::mutex mtx;

int main()
{   
    // initialize MPU6050  DRV8825
    MPU6050 MPU;
    
    DRV8825::SelectMotor(DRV8825::MOTOR1);
    DRV8825::Enable();

    DRV8825::SelectMotor(DRV8825::MOTOR2);
    DRV8825::Enable();

    //RUN thread using call back
    MPU.setCallback([](float pitch, float ax) {
        std::lock_guard<std::mutex> lock(mtx);

        float target_velocity = Vertical(0,pitch, ax);
    // Using PID
        float temp_left = Velocity(target_velocity);
        float temp_right = Velocity(target_velocity);

        DRV8825::SelectMotor(DRV8825::MOTOR1);
        driveMotorWithVelocityOutput(temp_left);
        DRV8825::SelectMotor(DRV8825::MOTOR2);
        driveMotorWithVelocityOutput(temp_right);
    });
    
    std::thread sensor_thread(&MPU6050::run, &MPU);
    // wait the thread finish
    sensor_thread.join();
    return 0;
}
