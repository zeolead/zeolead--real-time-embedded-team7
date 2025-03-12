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
    // low pass filter
    float low_pass_filter(float new_data, float old_data, float alpha);
    //read data from I2C
    int i2c_read_word(int addr);

public:
    MPU6050();  
    ~MPU6050(); 

    //initialize
    bool initialize();

    // Accelerometer and gyroscope bias calibration
    bool calibrate();

    //read data from accelerometer and gyroscope
    bool read_data(float &pitch, float &roll, float &ax);

    // apply low pass filter
    bool set_low_pass_filter();

    // Prints the sensor data to the console
    void print_data(float pitch, float roll, float ax);

    // Starts a thread for data reading and processing
    void run();
    
};
// Low pass filter function declaration
float low_pass_filter(float new_data, float old_data, float alpha);

// Function to read a word from I2C
int i2c_read_word(int fd, int addr);

#endif 
