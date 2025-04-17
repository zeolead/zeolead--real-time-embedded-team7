#include <iostream>
#include <thread>
#include <chrono>
#include "MotorControl.hpp"

int main() {
    MotorControl motor(DRV8825::MOTOR1);
    motor.setStatusCallback([](uint8_t dir, int rpm) {
        std::cout << "[Status] Direction=" << int(dir)
                  << ", RPM=" << rpm << std::endl;
    });
    motor.setStepCallback([]() {
        std::cout << "[Step] Pulse sent" << std::endl;
    });

    motor.start();
    motor.setRPM(100);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    motor.setRPM(-50);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    motor.stop();
    return 0;
}
