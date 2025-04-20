// main.cpp
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <csignal>
#include <chrono>

// 以下头文件请替换成你项目中的对应路径
#include "Webservercontroller.hpp"
#include "mpu6050_Kalman.hpp"
#include "PID.hpp"
#include "MotorControl.hpp"

// 全局退出标志
static std::atomic<bool> quit{false};

// Ctrl+C 信号处理：只设置退出标志，不 exit()
void Handler(int) {
    std::cout << "\n[Signal] Manual stop triggered\n";
    quit = true;
}

// 简单将字符串命令映射为数字
int casenumber(const std::string& cmd) {
    if (cmd == "forward")  return 0;
    if (cmd == "backward") return 1;
    if (cmd == "left")     return 2;
    if (cmd == "right")    return 3;
    if (cmd == "stop")     return 4;
    return -1;
}

int main() {
    // 1. 安装信号处理
    std::signal(SIGINT, Handler);

    // 2. 硬件初始化
    if (DEV_Config::DEV_ModuleInit()) {
        std::cerr << "[Error] DEV_ModuleInit failed\n";
        return 1;
    }

    MPU6050      mpu;
    PID          pid;
    MotorControl motor1(DRV8825::MOTOR1);
    MotorControl motor2(DRV8825::MOTOR2);

    // 3. 线程安全的消息队列，用于主线程接收 WebSocket 命令
    std::mutex              msg_mtx;
    std::condition_variable msg_cv;
    std::deque<std::string> msg_queue;

    // 4. 连接：MPU → PID → Motor
    mpu.setCallback([&](float pitch, float accel) {
        pid.receiveSensorData(pitch, accel);
    });
    pid.setOutputCallback([&](float output) {
        int rpm = static_cast<int>(output);
        motor1.setRPM(rpm);
        motor2.setRPM(-rpm);
    });

    // 5. 启动传感器线程
    std::thread sensorTh([&]() {
        mpu.run();  // 假设内部循环会一直跑，直到 mpu.stop() 被调用
    });

    // 6. 启动 WebSocket 服务
    Webservercontroller server;
    server.startServer([&](const std::string& cmd) {
        {
            std::lock_guard<std::mutex> lk(msg_mtx);
            msg_queue.push_back(cmd);
        }
        msg_cv.notify_one();
    });

    // run() 一般会阻塞在事件循环，故放到独立线程
    std::thread serverTh([&]() {
        server.run();  // 或者 endpoint.run()
    });

    std::cout << "[Info] System initialized, waiting for commands...\n";

    // 7. 主线程：等待并实时处理新命令
    std::string lastCmd;
    while (!quit.load()) {
        std::unique_lock<std::mutex> lk(msg_mtx);
        // 等待新消息或退出信号，超时后可重试
        msg_cv.wait_for(lk, std::chrono::milliseconds(50), [&]() {
            return !msg_queue.empty() || quit.load();
        });

        if (!msg_queue.empty()) {
            lastCmd = std::move(msg_queue.front());
            msg_queue.pop_front();
            lk.unlock();

            int cmdId = casenumber(lastCmd);
            switch (cmdId) {
                case 0: pid.receivePIDParams(0.3f, 0.1f, 0.2f, 0.05f,  3.0f); break;
                case 1: pid.receivePIDParams(0.3f, 0.1f, 0.2f, 0.05f, -3.0f); break;
                case 2: /* 左转逻辑 */ break;
                case 3: /* 右转逻辑 */ break;
                case 4: pid.receivePIDParams(0.3f, 0.1f, 0.2f, 0.05f,  0.0f); break;
                default:
                    std::cerr << "[Warn] Unknown cmd: " << lastCmd << "\n";
                    break;
            }

            std::cout << "[CMD] " << lastCmd << "\n";
        }
    }

    // 8. 关闭
    std::cout << "[Info] Shutting down...\n";

    // 停 WebSocket
    server.stopServer();
    if (serverTh.joinable()) serverTh.join();

    // 停 MPU 线程
    mpu.stop();  // 在类里实现让 run() 返回
    if (sensorTh.joinable()) sensorTh.join();

    // 停电机
    motor1.stop();
    motor2.stop();

    std::cout << "[Info] Exited cleanly\n";
    return 0;
}
