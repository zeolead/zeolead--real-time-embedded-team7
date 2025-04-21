#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <atomic>
#include "mpu6050_Kalman.hpp"
#include "PID.hpp"
#include "MotorControl.hpp"
#include "Webservercontroller.hpp"

static std::atomic<bool> quit{false};
//this function is designed to close all gpio
void Handler(int sig) {
    std::cout << "\nHandler: Manual Stop" << std::endl;
    DEV_Config::DEV_ModuleExit();
}

// the table of command from remote control
int casenumber(const std :: string& cmd){
        if (cmd == "forward") return 0;
        if (cmd == "backward") return 1;
        if (cmd == "left") return 2;
        if (cmd == "right") return 3;
        if (cmd == "stop") return 4;
        return -1;
        }


int main() {
    int turn=0;
    std::signal(SIGINT, Handler);
    if (DEV_Config::DEV_ModuleInit()) { //motor gpio initial
        return 0;
    }
    MPU6050 mpu;                                             //sensor
    PID pid;                                                         // algorithm
    MotorControl motor1(DRV8825::MOTOR1), //motor
                 motor2(DRV8825::MOTOR2);
    Webservercontroller server;                            //remote control
    std::string lastMsg;
    
    mpu.setCallback([&](float pitch, float ax) {   // sensor gives data to algorithm 
        pid.receiveSensorData(pitch, ax);
    });
    pid.setOutputCallback([&](float output) {      //algorithm gives speed command to motor
        int rpm = static_cast<int>(output);
        motor1.setRPM(rpm - turn);
        motor2.setRPM(-rpm - turn);
    });

    motor1.setStatusCallback([&](int direction, int rpm_) {             //motor give speed data to algorithm
        //std::cout << "RPM from callback:" << rpm_ << std::endl;
        pid.receiveRPM(rpm_);
        });

    motor1.start(); //motor thread
    motor2.start();
    
    
    server.setMessageCallback([&](const std::string& msg) {        //server command
        int cmd = casenumber(msg);
        Debug::Log("cmd %d\n",cmd);
        switch (cmd) {
            case 0:  //forward
            turn=0;
            pid.receivePIDParams(3.0f, 0.1f, 0.0f, 0.001f, 10.0f);
            std::cout<<"111111"<<std::endl;
            break; 
                
        case 1:    //backward
            turn=0;
            pid.receivePIDParams(3.0f, 0.1f, 0.0f, 0.001f, -10.0f);
            std::cout<<"111111"<<std::endl;
            break;  
                
        case 2: //turn left
            turn=5;
            pid.receivePIDParams(3.0f, 0.1f, 0.0f, 0.001f, 0.0f);
            std::cout<<"111111"<<std::endl;
            break; 
                
        case 3: //turn right
            turn=-5;
            pid.receivePIDParams(3.0f, 0.1f, 0.0f, 0.001f, 0.0f);
            std::cout<<"111111"<<std::endl;
            break; 
                
        case 4://on foot
            turn=0;
            pid.receivePIDParams(3.0f, 0.02f, 0.0f, 0.001f, 0.0f);
            std::cout<<"111111"<<std::endl;
            break; 
        default:
            std::cout<<"error"<<std::endl;
            break;
    }
        std::cout << "[CMD] ?? PID ??: " << msg << std::endl;
    });
    std::thread wsThread([&]{
        server.startServer();
    });
    wsThread.detach();
    
    mpu.run();
    server.stopServer();
    std::cout << "server stopped, lastMsg=" << lastMsg << std::endl;
    return 0;
}
    
