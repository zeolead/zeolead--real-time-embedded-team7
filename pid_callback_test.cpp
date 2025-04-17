#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include "PID.hpp"

int main() {
    PID pid;
    pid.setOutputCallback([](float output) {
        std::cout << "[PID Callback] Output = " << output << std::endl;
    });

    // Simulate sensor data: sine-wave pitch
    for (int i = 0; i < 100; ++i) {
        float pitch = std::sin(i * 0.1f) * 10.0f;
        float ay = 0.0f;
        pid.receiveSensorData(pitch, ay);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}