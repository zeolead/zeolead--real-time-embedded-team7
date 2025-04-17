#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <functional>
#include <vector>        // 添加，用于记录数据
#include <chrono>        // 添加，用于计时
#include <cstdio>        // 添加，用于 popen
#include "mpu6050_Kalman.hpp"

// ...（其它代码保持不变）...

void MPU6050::run() {
    file = open("/dev/i2c-1", O_RDWR);  // Open I2C device
    if (file < 0) {
        std::cerr << "Unable to open I2C device" << std::endl;
        return;
    }
    if (ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
        std::cerr << "Unable to connect to MPU6050" << std::endl;
        close(file);
        return;
    }

    // Wake up the MPU6050
    unsigned char wakeup[2] = {PWR_MGMT_1, 0};  
    write(file, wakeup, 2);

    // Apply Low pass filter 
    unsigned char dlpf_cfg[2] = {0x1A, 0x03};  
    write(file, dlpf_cfg, 2);

    // Accelerometer and gyroscope bias calibration
    float ay_offset = 0.0, az_offset = 0.0;
    float gx_offset = 0.0;
    const int num_samples = 100;
    for (int i = 0; i < num_samples; i++) {
        ay_offset += i2c_read_word(file, ACCEL_YOUT_H) / 16384.0;
        az_offset += i2c_read_word(file, ACCEL_ZOUT_H) / 16384.0;
        gx_offset += i2c_read_word(file, GYRO_XOUT_H) / 131.0;
        usleep(5000);   // 5ms 延时
    }
    ay_offset /= num_samples;
    az_offset /= num_samples;
    gx_offset /= num_samples;

    // 初始化卡尔曼滤波器
    KalmanFilter Kalman_pitch(0.03, 0.05, 0.01);
    float dt = 0.01;  // 10ms 时间间隔

    // 初始化数据记录容器（记录时间、滤波后 pitch 及加速度计角 accel_pitch）
    auto start_time = std::chrono::steady_clock::now();
    std::vector<double> timeVec, pitchVec, accelPitchVec;

    // 采集数据 20 秒
    while (true) {
        // 计算经过的时间（秒）
        auto current_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();
        if (elapsed >= 20.0) {
            break;
        }
    
