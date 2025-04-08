#include <iostream>
#include <csignal>
#include "MotorControl.hpp"
#include "DEV_Config.hpp"
#include "DRV8825.hpp"
#include "Debug.hpp"


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
    motor1.setRPM(80);
    //motor2.SetDirection(DRV8825::FORWARD);
    //motor2.setRPM(100);

    motor1.start();
    //motor2.start();

   // rpm   tune
   //  80      C4
   //  90      D4
   //2340     F5
    std::this_thread::sleep_for(std::chrono::seconds(1));

    motor1.setRPM(2340);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    //motor2.setRPM(2340);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    motor1.stop();
    //motor2.stop();

    //DEV_Config::DEV_ModuleExit();
    return 0;
}