// Kalman Filter
//           Gyroscope (drift) ----> Predicted angle
//                                  |
// Accelerometer (angle) -------> Correction (fusion) ----> Current estimated angle

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <functional>

#include <vector>        // Added for data storage
#include <chrono>        // Added for timing
#include <cstdio>        // Added for popen

#include "mpu6050_Kalman.hpp"

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B

#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43

// Constructor
MPU6050::MPU6050()
    : file(-1), ay_offset(0), az_offset(0), gx_offset(0), running_(false) {}  // Initialize the I2C file descriptor

// Destructor
MPU6050::~MPU6050() {
    stop();
    if (file >= 0) close(file);
}

void MPU6050::stop() {
    std::lock_guard<std::mutex> lock(data_mutex);
    running_ = false;
}

// Read a 16-bit word from I2C bus
int16_t MPU6050::i2c_read_word(int fd, int addr) {
    unsigned char buf[2];
    buf[0] = addr;
    if (write(fd, buf, 1) != 1) return -1;
    if (read(fd, buf, 2) != 2) return -1;
    return (int16_t)(buf[0] << 8) | buf[1];
}

// Kalman Filter structure
typedef struct KalmanFilter {
    float Q_angle;   // Process noise variance for the accelerometer (angle change)
    float Q_gyro;    // Process noise variance for the gyro drift
    float R_angle;   // Measurement noise variance (accelerometer)
    float angle;     // Current angle estimate
    float bias;      // Current bias estimate
    float P[2][2];   // Error covariance matrix

    KalmanFilter(float Q_angle, float Q_gyro, float R_angle)
        : Q_angle(Q_angle), Q_gyro(Q_gyro), R_angle(R_angle), angle(0), bias(0) {
        P[0][0] = 1; P[0][1] = 0;
        P[1][0] = 0; P[1][1] = 1;
    }

    // Update the Kalman filter
    float update(float newAngle, float newRate, float dt) {
        // Prediction step
        angle += dt * (newRate - bias);
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_gyro * dt;

        // Update step
        float S = P[0][0] + R_angle;
        float K[2] = { P[0][0] / S, P[1][0] / S };
        float y = newAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;

        // Update error covariance matrix
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }
} KalmanFilter;

// Low-pass filter
float MPU6050::low_pass_filter(float new_data, float old_data, float alpha) {
    return alpha * new_data + (1.0f - alpha) * old_data;
}

void MPU6050::run() {
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        running_ = true;
    }
    // Open I2C device
    file = open("/dev/i2c-1", O_RDWR);
    if (file < 0) {
        std::cerr << "Unable to open I2C device" << std::endl;
        return;
    }
    // Connect to MPU6050
    if (ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
        std::cerr << "Unable to connect to MPU6050" << std::endl;
        close(file);
        return;
    }

    // Wake up the MPU6050
    unsigned char wakeup[2] = {PWR_MGMT_1, 0};
    write(file, wakeup, 2);

    // Apply low-pass filter
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
        usleep(5000);   // 5ms delay
    }
    ay_offset /= num_samples;
    az_offset /= num_samples;
    gx_offset /= num_samples;

    // Initialize Kalman filter and timing
    KalmanFilter Kalman_pitch(0.03, 0.05, 0.01);
    float dt = 0.01;  // Time interval 10ms

    // Initialize data containers (records time, filtered pitch, and accel pitch)
    auto start_time = std::chrono::steady_clock::now();
    std::vector<double> timeVec, pitchVec, accelPitchVec;

    // Collect data for 20 seconds
    while (true) {
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (!running_) break;
        }
        // Calculate elapsed time in seconds
        auto current_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();
        // if (elapsed >= 20.0) {
        //     break;
        // }

        // Read data
        int accel_y = i2c_read_word(file, ACCEL_YOUT_H);
        int accel_z = i2c_read_word(file, ACCEL_ZOUT_H);
        int gyro_x  = i2c_read_word(file, GYRO_XOUT_H);

        // Convert raw data to g and degrees/s
        float ay = accel_y / 16384.0 - ay_offset;
        float az = -(accel_z / 16384.0 - az_offset);
        float gx = (gyro_x / 131.0) - gx_offset;

        // Calculate accelerometer tilt angles （Pitch and Roll）
        float accel_pitch = atan2(accel_y / 16384.0 - 0.01, -accel_z / 16384.0 - 0.17) * 180.0 / M_PI;  // Artificially add the offset value

        // Kalman filter: refresh accelerometer
        float pitch = Kalman_pitch.update(accel_pitch, gx, dt);  //gyro_pitch
        pitch = std::clamp(pitch, -90.0f, 90.0f);

        // Record data (time, filtered pitch, and accel_pitch)
        timeVec.push_back(elapsed);
        pitchVec.push_back(pitch);
        accelPitchVec.push_back(accel_pitch);

        // Output data to console (optional)
        {
            std::lock_guard<std::mutex> guard(data_mutex);
            std::cout << "Time: " << elapsed << " s, "
                      << "Pitch (Kalman): " << pitch << "° " << std::endl;
            std::cout << "Accel Pitch: " << accel_pitch << "°, "
                      << "F/B acceleration (ay): " << ay << " g" << std::endl;
        }

        // If callback exists (e.g., for PID control), still call
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (callback) callback(pitch, ay);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // usleep(10000);  // Delay 10ms
    }

    //Output the calculated pitch and the forward/backward acceleration (ax)
    //    std::lock_guard<std::mutex> guard(data_mutex);  // Mutex to protect shared std::cout
    //    std::cout << "Pitch: " << pitch << "°  F/B acceleration (ay): " << ay << " g" << std::endl;
    //    std::cout << "gx: " << gx << "  a_p " << accel_pitch << std::endl;


    
    // After collection, call gnuplot to plot the graph
    // FILE* gp = popen("gnuplot -persistent", "w");
    // if (!gp) {
    //     std::cerr << "Error: could not open gnuplot." << std::endl;
    // } else {
    //     fprintf(gp, "set title 'Pitch and Accel_Pitch vs Time'\n");
    //     fprintf(gp, "set xlabel 'Time (s)'\n");
    //     fprintf(gp, "set ylabel 'Angle (°)'\n");
    //     fprintf(gp, "set grid\n");
    //     // Plot two curves: first for filtered pitch, second for accel_pitch
    //     fprintf(gp, "plot '-' with lines title 'Filtered Pitch', '-' with lines title 'Accel Pitch'\n");

    //     // Send first data set (time and filtered pitch)
    //     for (size_t i = 0; i < timeVec.size(); i++) {
    //         fprintf(gp, "%f %f\n", timeVec[i], pitchVec[i]);
    //     }
    //     fprintf(gp, "e\n");

    //     // Send second data set (time and accel_pitch)
    //     for (size_t i = 0; i < timeVec.size(); i++) {
    //         fprintf(gp, "%f %f\n", timeVec[i], accelPitchVec[i]);
    //     }
    //     fprintf(gp, "e\n");

    //     pclose(gp);
    // }

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

    // Wait for the sensor thread to finish
    sensor_thread.join();

    return 0;
}
