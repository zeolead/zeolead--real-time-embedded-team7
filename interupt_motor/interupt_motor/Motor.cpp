#include <iostream>
#include <csignal>
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

int main() {
    //1.System Initialization
    if (DEV_Config::DEV_ModuleInit()) {
        return 0;
    }

    // Exception handling:ctrl + c
    std::signal(SIGINT, Handler);

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

    while (true) {
        DRV8825::SelectMotor(DRV8825::MOTOR1);
        DRV8825::SetMicroStep(DRV8825::HARDWARE, "1/8step");
        DRV8825::TurnStep(DRV8825::BACKWARD, 1600, 32);
        DRV8825::Stop();

        DRV8825::SelectMotor(DRV8825::MOTOR2);
        DRV8825::SetMicroStep(DRV8825::HARDWARE, "1/8step");
        DRV8825::TurnStep(DRV8825::BACKWARD, 1600, 32);
        DRV8825::Stop();
    }

    DEV_Config::DEV_ModuleExit();
    return 0;
}
