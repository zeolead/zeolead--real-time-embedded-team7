// mpu6050_test001.h

#ifndef mpu6050_test001_H  
#define mpu6050_test001_H

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <thread>
#include <functional>    
#include <mutex>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

class MPU6050 {
private:
    int file;    // I2C device file descriptor

    // Offsets for accelerometer and gyroscope
    float ax_offset, ay_offset, az_offset;
    float gx_offset, gy_offset, gz_offset;
    float pitch;
    
    // Low pass filter function
    float low_pass_filter(float new_data, float old_data, float alpha);
    // Read data from I2C
    int i2c_read_word(int fd, int addr);

    // Mutex for protecting shared data
    std::mutex data_mutex;  

    // Callback function to process sensor data (parameters: pitch and ax)
    std::function<void(float, float)> callback;  

public:
    MPU6050();  
    ~MPU6050(); 

    // Starts the data reading thread
    void run();
    // Existing interface to read data
    void readdata(float &pitch, float &ax);

    // Setter for the callback interface
    void setCallback(std::function<void(float, float)> cb) {
        std::lock_guard<std::mutex> lock(data_mutex);
        callback = cb;
    }
};

int sensor_start();

#endif
