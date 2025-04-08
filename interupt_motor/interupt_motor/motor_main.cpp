#include <iostream>
#include <csignal>
#include "MotorControl.hpp"
#include "DEV_Config.hpp"
#include "DRV8825.hpp"
#include "Debug.hpp"

void Handler(int signo) {
    std::cout << "\nHandler: Motor Stop" << std::endl;
    DRV8825::SelectMotor(DRV8825::MOTOR1);
    DRV8825::Stop();
    DRV8825::SelectMotor(DRV8825::MOTOR2);
    DRV8825::Stop();
    DEV_Config::DEV_ModuleExit();
    exit(0);
}

/*
# 1.8 degree: nema23, nema14
# softward Control :
# 'fullstep': A cycle = 200 steps
# 'halfstep': A cycle = 200 * 2 steps
# '1/4step': A cycle = 200 * 4 steps
# '1/8step': A cycle = 200 * 8 steps
# '1/16step': A cycle = 200 * 16 steps
# '1/32step': A cycle = 200 * 32 steps
*/

int main() {
    //1.System Initialization
    if (DEV_Config::DEV_ModuleInit()) {
        return 0;
    }

    MotorControl motor1(DRV8825::MOTOR1);
    MotorControl motor2(DRV8825::MOTOR2);

    motor1.SetDirection(DRV8825::BACKWARD);
    motor1.setRPM(30);
    motor2.SetDirection(DRV8825::FORWARD);
    motor2.setRPM(60);

    motor1.start();
    motor2.start();

    std::this_thread::sleep_for(std::chrono::seconds(5));

    motor1.setRPM(100);
    motor2.setRPM(20);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    motor1.stop();
    motor2.stop();

    DEV_Config::DEV_ModuleExit();
    return 0;
}