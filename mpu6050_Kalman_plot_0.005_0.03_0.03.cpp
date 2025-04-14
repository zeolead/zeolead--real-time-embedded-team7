
#include <fstream>  //用于文件操作
#include <iostream> //控制台输入输出
#include <fcntl.h>  //open（）函数
#include <unistd.h> //read（），write（），close()
#include <sys/ioctl.h>  //ioctl()函数
#include <linux/i2c-dev.h>  //I2C设备控制接口
#include <cmath>    //数学库，如sqrt、atan2、M_PI等

#define MPU6050_ADDR 0x68   //默认I2C地址，MPU6050的WHO_AM_I返回也是0x68
#define PWR_MGMT_1 0x6B //电源管理寄存器，用于唤醒传感器

#define ACCEL_YOUT_H 0x3D   //加速度Y轴高8位的寄存器地址
#define ACCEL_ZOUT_H 0x3F   //加速度z轴高8位的寄存器地址
#define GYRO_XOUT_H  0x43   //陀螺仪X轴高8位的寄存器地址


//从指定寄存器地址读取16位有符号数据（高位在前，低位在后）
int16_t i2c_read_word(int fd, int addr) 
{
    unsigned char buf[2];
    buf[0] = addr;
    if (write(fd, buf, 1) != 1) return -1;
    if (read(fd, buf, 2) != 2) return -1;

    return (int16_t)((buf[0] << 8) | buf[1]);
}



// 卡尔曼滤波器结构
struct KalmanFilter {
    float Q_angle;  // 过程噪声方差（加速度计）
    float Q_gyro;   // 过程噪声方差（陀螺仪）
    float R_angle;  // 测量噪声方差
    float angle;     // 当前角度估算
    float bias;      // 当前偏差估算
    float P[2][2];   // 误差协方差矩阵

    KalmanFilter(float Q_angle, float Q_gyro, float R_angle)
        : Q_angle(Q_angle), Q_gyro(Q_gyro), R_angle(R_angle), angle(0), bias(0) {
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



int main() {
    int file = open("/dev/i2c-1", O_RDWR);  // Open I2C device
    if (file < 0) {
        std::cerr << "Unable to open I2C device" << std::endl;
        return 1;
    }
    if (ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
        std::cerr << "Unable to connect to MPU6050" << std::endl;
        return 1;
    }


    std::ofstream dataFile("data.csv");
    dataFile << "Pitch,ax\n";


    // Wake up the MPU6050
    unsigned char wakeup[2] = {PWR_MGMT_1, 0};  // Wake up MPU6050
    write(file, wakeup, 2);

    // Low pass filter configuration
    unsigned char dlpf_cfg[2] = {0x1A, 0x03};  // Set low pass filter
    write(file, dlpf_cfg, 2);

    // Accelerometer and gyroscope bias calibration
    float ay_offset = 0.0, az_offset = 0.0;
    float gx_offset = 0.0;
    const int num_samples = 100;
    for (int i = 0; i < num_samples; i++) {
        ay_offset += i2c_read_word(file, ACCELY_XOUT_H) / 16384.0;
        az_offset += i2c_read_word(file, ACCEL_ZOUT_H) / 16384.0;
        gx_offset += i2c_read_word(file, GYRO_XOUT_H) / 131.0;
        usleep(5000);
    }
    ay_offset /= num_samples;
    az_offset /= num_samples;
    gx_offset /= num_samples;


    // 初始化卡尔曼滤波器
    KalmanFilter kalman_pitch(0.005, 0.03, 0.03);  // 适当调整这些参数

    // 初始化其他变量
    float dt = 0.01;  // 时间间隔 10ms


    // Start reading data and calculating
    while (true) {
        // Read accelerometer data
        int accel_y = i2c_read_word(file, ACCEL_XOUT_H + 2);
        int accel_z = i2c_read_word(file, ACCEL_XOUT_H + 4);

        // Read gyroscope data
        int gyro_x = i2c_read_word(file, GYRO_XOUT_H);

        // Convert raw data to units of g and °/s
        float ay = accel_y / 16384.0 - ay_offset;
        float az = accel_z / 16384.0 - az_offset;

        float gx = (gyro_x / 131.0) - gx_offset;

        // Calculate accelerometer tilt angles （Pitch and Roll）
        float accel_pitch = atan2(ay, az) * 180.0 / M_PI;

        // Use gyroscope angular velocity to update angles
        float gyro_pitch = gx * dt;

        // 使用卡尔曼滤波器更新角度
        float pitch = kalman_pitch.update(accel_pitch, gyro_pitch, dt);

        // 保存到 CSV 文件并输出
        dataFile << pitch << "," << ax << "\n";
        std::cout << "Pitch: " << pitch << "°  F/B acceleration (ax): " << ax << std::endl;

        usleep(10000);  // Delay 10ms
    }

    dataFile.close();
    close(file);
    return 0;
}

