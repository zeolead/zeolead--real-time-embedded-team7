#include <iostream>
#include <thread>
#include <csignal>
#include <atomic>
#include <chrono>

#include "mpu6050_Kalman.hpp"
#include "PID.hpp"
#include "MotorControl.hpp"
#include "Webservercontroller.hpp"

// 全局退出标志
static std::atomic<bool> quit{false};

// CTRL+C 信号处理
void Handler(int) {
    std::cout << "\nManual stop\n";
    DEV_Config::DEV_ModuleExit();
    quit = true;
}

int casenumber(const std::string& cmd){
    if (cmd=="forward")  return 0;
    if (cmd=="backward") return 1;
    if (cmd=="left")     return 2;
    if (cmd=="right")    return 3;
    if (cmd=="stop")     return 4;
    return -1;
}

int main(){
    // 安装信号
    std::signal(SIGINT, Handler);

    // 1. 硬件初始化
    if (DEV_Config::DEV_ModuleInit()) return 1;

    MPU6050 mpu;
    PID pid;
    MotorControl motor1(DRV8825::MOTOR1);
    MotorControl motor2(DRV8825::MOTOR2);

    // 1.1 传感器 → PID
    mpu.setCallback([&](float pitch, float ax){
        pid.receiveSensorData(pitch, ax);
    });

    // 1.2 PID → 电机
    pid.setOutputCallback([&](float output){
        int rpm = static_cast<int>(output);
        motor1.setRPM(rpm);
        motor2.setRPM(-rpm);
    });

    // 1.3 启动线程
    std::thread mpuThread(&MPU6050::run, &mpu);
    motor1.start();
    motor2.start();

    // 2. 启动 WebSocket 服务
    Webservercontroller server;
    server.startServer([&](const std::string& msg){
        int cmd = casenumber(msg);
        switch(cmd){
          case 0: pid.receivePIDParams(0.3f,0.1f,0.2f,0.05f,  3.0f); break;
          case 1: pid.receivePIDParams(0.3f,0.1f,0.2f,0.05f, -3.0f); break;
          case 2: /* 左转 */ break;
          case 3: /* 右转 */ break;
          case 4: pid.receivePIDParams(0.3f,0.1f,0.2f,0.05f,  0.0f); break;
          default: std::cerr<<"Unknown cmd\n"; break;
        }
        std::cout<<"recv: "<<msg<<std::endl;
    });

    std::cout<<"Server up, motors spinning...\n";

    // 3. 等待退出
    while(!quit){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 4. 优雅收尾
    server.stopServer();
    mpu.stop();          // 假设你的 MPU 类提供停止接口
    if(mpuThread.joinable()) mpuThread.join();
    motor1.stop();       // 如果有 stop
    motor2.stop();

    std::cout<<"Exited cleanly\n";
    return 0;
}
