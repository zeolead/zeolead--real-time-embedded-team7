#ifndef PID_HPP
#define PID_HPP

#include <functional>

class MPU6050;

class PID {
public:

    float Vertical(float Expect, float Angle, float gyro_x);
    float Velocity(float targetSpeed);
   
    //read data from sensor
    void receiveSensorData(float pitch, float ax, float gx);
    //set callback
    void setOutputCallback(std::function<void(float)> callback);
private:
    std::function<void(float)> outputCallback;
};

#endif // PID_HPP
