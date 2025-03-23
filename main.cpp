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

// protect data
std::mutex mtx;

int main()
{   
    MPU6050 MPU;
    
    // Register the callback interface: when new data is available, the callback is invoked
    MPU.setCallback([](float pitch, float ax) {
        int pid_output = Vertical(0, pitch, ax);
        std::cout << "PID output: " << pid_output << std::endl;
    });

    // Run the sensor data reading thread
    std::thread sensor_thread(&MPU6050::run, &MPU);
    sensor_thread.join();

    return 0;
}