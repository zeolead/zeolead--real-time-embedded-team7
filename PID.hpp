#ifndef PID_HPP
#define PID_HPP

#include <functional>

class MPU6050;

class PID {
public:

    float Vertical(float angle_output,float pitch, float ax);
    float Velocity();
   
    //read data from sensor
    void receiveSensorData(float pitch, float ax);
    //set callback
    void setOutputCallback(std::function<void(float)> callback);
private:
    std::function<void(float)> outputCallback;
    float current_pitch = 0.0f;
};

#endif // PID_HPP
