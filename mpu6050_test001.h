#ifndef mpu6050_test001_H  
#define mpu6050_test001_H

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include<thread>


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
    // low pass filter
    float low_pass_filter(float new_data, float old_data, float alpha);
    //read data from I2C
    int i2c_read_word(int fd, int addr);
    //data locker
    std::mutex data_mutex;  
    

public:
    MPU6050();  
    ~MPU6050(); 

    // Starts a thread for data reading and processing
    void run();
    void readdata(float &pitch, float &ax);
};
// Function to start the sensor data reading thread
int sensor_start();

#endif 
