#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <thread>
#include <mutex>
#include"mpu6050_test001.h"

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43


// Constructor
MPU6050::MPU6050() {
    file = -1;  // Initialize the I2C file descriptor
    ax_offset = ay_offset = az_offset = 0.0f;
    gx_offset = gy_offset = gz_offset = 0.0f;
}

// Destructor
MPU6050::~MPU6050() {
    if (file >= 0) {
        close(file);
    }
}
int MPU6050::i2c_read_word(int fd, int addr) {
    unsigned char buf[2];
    buf[0] = addr;
    if (write(fd, buf, 1) != 1) return -1;
    if (read(fd, buf, 2) != 2) return -1;
    return (buf[0] << 8) | buf[1];
}

// Low pass filter
float MPU6050::low_pass_filter(float new_data, float old_data, float alpha) {
    return alpha * new_data + (1.0f - alpha) * old_data;
}

// Mutex to protect shared data (std::cout)
extern std::mutex mtx;

void MPU6050::run() {
    int file = open("/dev/i2c-1", O_RDWR);  // Open I2C device
    if (file < 0) {
        std::cerr << "Unable to open I2C device" << std::endl;
        return;
    }
    if (ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
        std::cerr << "Unable to connect to MPU6050" << std::endl;
        return;
    }

    // Wake up the MPU6050
    unsigned char wakeup[2] = {PWR_MGMT_1, 0};  
    write(file, wakeup, 2);

    //Apply  Low pass filter 
    unsigned char dlpf_cfg[2] = {0x1A, 0x03};  
    write(file, dlpf_cfg, 2);

    // Accelerometer and gyroscope bias calibration
    float ax_offset = 0.0, ay_offset = 0.0, az_offset = 0.0;
    float gx_offset = 0.0, gy_offset = 0.0, gz_offset = 0.0;
    const int num_samples = 100;
    for (int i = 0; i < num_samples; i++) {
        ax_offset += i2c_read_word(file, ACCEL_XOUT_H) / 16384.0;
        ay_offset += i2c_read_word(file, ACCEL_XOUT_H + 2) / 16384.0;
        az_offset += i2c_read_word(file, ACCEL_XOUT_H + 4) / 16384.0;
        gx_offset += i2c_read_word(file, GYRO_XOUT_H) / 131.0;
        gy_offset += i2c_read_word(file, GYRO_XOUT_H + 2) / 131.0;
        gz_offset += i2c_read_word(file, GYRO_XOUT_H + 4) / 131.0;
        usleep(5000);
    }
    ax_offset /= num_samples;
    ay_offset /= num_samples;
    az_offset /= num_samples;
    gx_offset /= num_samples;
    gy_offset /= num_samples;
    gz_offset /= num_samples;

    // Initialize 
    float pitch = 0.0, roll = 0.0;
    float gyro_pitch = 0.0, gyro_roll = 0.0;
    float dt = 0.01;  // Time interval 10ms

    //Initialize low pass filter parameters
    float prev_pitch = 0.0, prev_roll = 0.0;
    const float FILTER_ALPHA = 0.7;  // Adjust accelerometer and gyroscope weights

    // Start reading data and calculating
    while (true) {
        // Read  data
        int accel_x = i2c_read_word(file, ACCEL_XOUT_H);
        int accel_y = i2c_read_word(file, ACCEL_XOUT_H + 2);
        int accel_z = i2c_read_word(file, ACCEL_XOUT_H + 4);
        int gyro_x = i2c_read_word(file, GYRO_XOUT_H);
        int gyro_y = i2c_read_word(file, GYRO_XOUT_H + 2);
        int gyro_z = i2c_read_word(file, GYRO_XOUT_H + 4);

        // Convert data 
        float ax = accel_x / 16384.0 - ax_offset;
        float ay = accel_y / 16384.0 - ay_offset;
        float az = accel_z / 16384.0 - az_offset;
        float gx = (gyro_x / 131.0) - gx_offset;
        float gy = (gyro_y / 131.0) - gy_offset;
        float gz = (gyro_z / 131.0) - gz_offset;

        // Calculate accelerometer tilt angles （Pitch and Roll）
        float accel_pitch = atan2(ay, az) * 180.0 / M_PI;
        float accel_roll = atan2(ax, az) * 180.0 / M_PI;

        // Use gyroscope angular velocity to update angles
        gyro_pitch += gx * dt;
        gyro_roll += gy * dt;

        // Complementary filter: combine accelerometer and gyroscope data
        pitch = 0.90 * (accel_pitch) + 0.10 * (gyro_pitch);  // Increase accelerometer weight
        roll = 0.90 * (accel_roll) + 0.10 * (gyro_roll);

        // Smoothing
        pitch = low_pass_filter(pitch, prev_pitch, FILTER_ALPHA);
        roll = low_pass_filter(roll, prev_roll, FILTER_ALPHA);

        prev_pitch = pitch;
        prev_roll = roll;

        // Output the calculated pitch and the forward/backward acceleration (ax)
        std::lock_guard<std::mutex> guard(mtx);  // Mutex to protect shared std::cout
        std::cout << "Pitch: " << pitch << "°  F/B acceleration (ax): " << ax << " g" << std::endl;

        usleep(500000);  // Delay 500ms
    }

    close(file);
}

int sensor_start() {
    MPU6050 sensor;  // Create an MPU6050 object
    // Start a new thread to run the sensor data reading and processing
    std::thread sensor_thread(&MPU6050::run, &sensor);

    // Optionally, you can join the thread or perform other tasks in the main thread
    sensor_thread.join();  // Wait for the sensor thread to finish

    return 0;
}
