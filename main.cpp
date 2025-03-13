#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include<gpiod.h>
#include<mutex>
#include"mpu6050_test001.h"

// protect data
std::mutex mtx;

int main()
{   
    MPU6050 MPU;
    
    //RUN thread
    std::thread sensor_thread(&MPU6050::run, &MPU);
    // wait the thread finish
    sensor_thread.join();
    return 0;
}
