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
void Handler(int sig) {
    std::cout << "\nHandler: Manual Stop" << std::endl;
    DEV_Config::DEV_ModuleExit();
    quit = true;
}
int casenumber(const std :: string& cmd){
        if (cmd == "forward") return 0;
        if (cmd == "backward") return 1;
        if (cmd == "left") return 2;
        if (cmd == "right") return 3;
        if (cmd == "stop") return 4;
        return -1;
        }


int main() {
    std::signal(SIGINT, Handler);
    Webservercontroller server;
    std::string lastMsg;

    std::cout <<"received" << lastMsg << std::endl;   
    //Webservercontroller server;
    //std::string lastMsg;
    //server.control([&](const std::string& m){
    //      lastMsg = m;
    //std::cout <<"514" << lastMsg << std::endl;
    //    });
    
    //std::this_thread::sleep_for(std::chrono::hours(24));
    
    int turn1=0;
    int turn2=0;
    
    if (DEV_Config::DEV_ModuleInit()) {
        return 0;
    }
    std::signal(SIGINT,Handler);
    
    
    MPU6050 mpu;
    PID pid;

    MotorControl motor1(DRV8825::MOTOR1);
    MotorControl motor2(DRV8825::MOTOR2);
    
    

    mpu.setCallback([&](float pitch, float ax) {
        pid.receiveSensorData(pitch, ax); 
         
    });
    
    pid.setOutputCallback([&](float output) {
        int rpm = static_cast<int>(output);
        motor1.setRPM(rpm+turn1);
        motor2.setRPM(-rpm+turn2);
       
    });
    server.setMessageCallback([&lastMsg](const std::string& msg){
        lastMsg = msg;
        std::cout <<"received message" << lastMsg << std::endl;
    }       
        server.startServer(const std::string& msg)
        
        
    };   
    int command =casenumber(lastMsg);
    std::cout <<"COMMAND" << lastMsg << std::endl;   
    
    switch (command)
    {
        case 0:
            turn1=0;
            turn2=0;
            pid.setParamCallback([&](){
                pid.receivePIDParams(0.3f, 0.1f, 0.2f, 0.05f, 3.0f);
                std::cout<<"111111"<<std::endl;
            });
            break; 
                
        case 1:
            turn1=0;
            turn2=0;
            pid.setParamCallback([&](){
                pid.receivePIDParams(0.3f, 0.1f, 0.2f, 0.05f, -3.0f);
                std::cout<<"111111"<<std::endl;
            });
            break;  
                
        case 2:
            turn1=5;
            turn2=0;
            pid.setParamCallback([&](){
                pid.receivePIDParams(0.3f, 0.1f, 0.2f, 0.05f, 3.0f);
                std::cout<<"111111"<<std::endl;
            });
            break; 
                
        case 3:
            turn1=0;
            turn2=5;
            pid.setParamCallback([&](){
                pid.receivePIDParams(0.3f, 0.1f, 0.2f, 0.05f, 3.0f);
                std::cout<<"111111"<<std::endl;
            });
            break; 
                
        case 4:
            turn1=0;
            turn2=0;
            pid.setParamCallback([&](){
                pid.receivePIDParams(0.3f, 0.1f, 0.2f, 0.05f, 0.0f);
                std::cout<<"111111"<<std::endl;
            });
            break; 
        default:
            std::cout<<"error"<<std::endl;
            break;
    }
    pid.triggerParamCallback();
    motor1.start();
    motor2.start();

    std::thread mpuThread(&MPU6050::run, &mpu);
    mpuThread.join(); 
    server.stopServer();
 
    //while(!quit){
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //} 
    std::cout << "close system" << std::endl;
    
    std::cout <<"server stopped" << lastMsg << std::endl;
    return 0; 
}


