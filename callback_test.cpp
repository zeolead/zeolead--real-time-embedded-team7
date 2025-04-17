// callback_test.cpp

#include <iostream>
#include <thread>
#include <chrono>
#include "mpu6050_Kalman.hpp"

int main() {
    MPU6050 sensor;

    // Set the callback function for testing:
    // When new sensor data (pitch and ax) is available, the callback will print them.
    sensor.setCallback([](float pitch, float ay) {
        std::cout << "[Callback Test] New data received: pitch = " << pitch 
                  << "°, ay = " << ay << "g" << std::endl;
    });

    // Start the sensor data reading thread
    std::thread sensorThread(&MPU6050::run, &sensor);

    // For testing, let the sensor run for a few iterations.
    // Here we sleep for 3 seconds to observe several callback outputs.
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "Exiting callback test program." << std::endl;

    sensorThread.detach();
    return 0;
}
