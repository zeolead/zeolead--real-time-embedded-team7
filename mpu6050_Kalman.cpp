// Kalman Filter
//           陀螺仪（漂移） ----> 预测角度
//                                  |
// 测量角度（加速度） ---------> 校正（融合）--------> 当前估计角度

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <functional>
#include"mpu6050_test001.hpp"

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B

#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43


// Constructor
MPU6050::MPU6050() {
    file = -1;  // Initialize the I2C file descriptor
    ay_offset = az_offset = 0.0f;
    gx_offset = 0.0f;
}

// Destructor
MPU6050::~MPU6050() {
    if (file >= 0) {
        close(file);
    }
}
int16_t MPU6050::i2c_read_word(int fd, int addr) {
    unsigned char buf[2];
    buf[0] = addr;
    if (write(fd, buf, 1) != 1) return -1;
    if (read(fd, buf, 2) != 2) return -1;
    return (int16_t)(buf[0] << 8) | buf[1];
}


// 卡尔曼滤波器结构
struct KalmanFilter {
    float Q_angle;  // （角度变化）过程噪声方差（加速度计）
    float Q_gyro;   // 过程噪声方差（陀螺仪漂移过程噪声）
    float R_angle;  // 测量（加速度计）噪声方差
    float angle;     // 当前角度估算
    float bias;      // 当前偏差估算
    float P[2][2];   // 误差协方差矩阵

    KalmanFilter(float Q_angle, float Q_gyro, float R_angle)
        : Q_angle(Q_angle), Q_gyro(Q_gyro), R_angle(R_angle), angle(0), bias(0){
        P[0][0] = 1; P[0][1] = 0;
        P[1][0] = 0; P[1][1] = 1;
    }



    // 更新卡尔曼滤波器
    float update(float newAngle, float newRate, float dt) {
        // 预测步骤
        angle += dt * (newRate - bias);
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_gyro * dt;

        // 更新步骤
        float S = P[0][0] + R_angle;
        float K[2] = { P[0][0] / S, P[1][0] / S };
        float y = newAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;

        // 更新误差协方差矩阵
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }
};


// Low pass filter
float MPU6050::low_pass_filter(float new_data, float old_data, float alpha) {
    return alpha * new_data + (1.0f - alpha) * old_data;
}


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

    //Apply  Low pass filter 
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
        usleep(5000);   //2000
    }
    ay_offset /= num_samples;
    az_offset /= num_samples;
    gx_offset /= num_samples;

    // Initialize 
    // float pitch = 0.0, roll = 0.0;
    // float gyro_pitch = 0.0, gyro_roll = 0.0;
    
    // Initialize Kalman Filter
    KalmanFilter Kalman_pitch(0.03, 0.05, 0.01);    //0.001， 0.003， 0.03
    
    float dt = 0.01;  // Time interval 10ms

    // Start reading data and calculating
    while (true) {
        // Read  data
        int accel_y = i2c_read_word(file, ACCEL_YOUT_H);
        int accel_z = i2c_read_word(file, ACCEL_ZOUT_H);
        int gyro_x = i2c_read_word(file, GYRO_XOUT_H);

        // Convert raw data to g and °/s
        float ay = accel_y / 16384.0 - ay_offset;
        float az = -（accel_z / 16384.0 - az_offset）;
        float gx = (gyro_x / 131.0) - gx_offset;

        // Calculate accelerometer tilt angles （Pitch and Roll）
        float accel_pitch = atan2(accel_y-0.01, -accel_z-0.17) * 180.0 / M_PI;  // Artificially add the offset value

        // Kalman filter: refresh accelerometer
        float pitch = Kalman_pitch.update(accel_pitch, gx, dt);  //gyro_pitch

        prev_pitch = pitch;

        // Output the calculated pitch and the forward/backward acceleration (ax)
        std::lock_guard<std::mutex> guard(data_mutex);  // Mutex to protect shared std::cout
        std::cout << "Pitch: " << pitch << "°  F/B acceleration (ay): " << ay << " g" << std::endl;
        std::cout << "gx: " << gx << "°  a_p " << accel_pitch << std::endl;
        std::cout << "az: " << az << "  az_offset " << az_offset <<  "  accel_z " << -accel_z/16384.0 << std::endl;
        std::cout << "ay: " << ay << "  ay_offset " << ay_offset << "  accel_y " << accel_y/16384.0 << std::endl;
        
        if (callback) callback(pitch, ay);
        usleep(10000);  // Delay 10ms
    }

    close(file);
}
void MPU6050::setCallback(const std::function<void(float, float)>& cb) {
    std::lock_guard<std::mutex> lock(data_mutex);
    callback = cb;
}

int sensor_start() {
    MPU6050 sensor;  // Create an MPU6050 object
    // Start a new thread to run the sensor data reading and processing
    std::thread sensor_thread(&MPU6050::run, &sensor);

    // Optionally, you can join the thread or perform other tasks in the main thread
    sensor_thread.join();  // Wait for the sensor thread to finish

    return 0;
}
