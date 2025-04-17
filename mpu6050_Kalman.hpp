#ifndef mpu6050_Kalman_H  
#define mpu6050_Kalman_H

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include<thread>
#include <functional>
#include <mutex>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

//class declaration
class MPU6050 {
private:
    int file;    // I2C device file descriptor

    // Offsets for accelerometer and gyroscope
    float ax_offset, ay_offset, az_offset;
    float gx_offset, gy_offset, gz_offset;
    float pitch;
    
    // callback
    std::function<void(float, float)> callback;  
    //data locker
    std::mutex data_mutex;
    bool running_;

    // I2C helpers
    //read data from I2C
    int16_t  i2c_read_word(int fd, int addr);      
    // low pass filter
    float low_pass_filter(float new_data, float old_data, float alpha);    

public:
    MPU6050();  
    ~MPU6050(); 

    // Starts a thread for data reading and processing
    void run();
    // Stop the run loop gracefully
    void stop();
    // Register a callback: pitch (°) and ay (g)
    void setCallback(const std::function<void(float, float )>& cb);
};
// Function to start the sensor data reading thread
int sensor_start();

#endif 
