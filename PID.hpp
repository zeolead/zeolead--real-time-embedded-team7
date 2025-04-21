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
    //read rpm from main
    void receiveRPM(float rpm);
    //set callback
    void setOutputCallback(std::function<void(float)> callback);

    void setParamCallback(std::function<void()> callback);
    void triggerParamCallback();
    void receivePIDParams(float vertical_kp,  float vertical_kd ,float velocity_kp  ,float velocity_ki, float target_velocity );

private:
    std::function<void()> paramCallback;
    std::function<void(float)> outputCallback;
    float current_pitch = 0.0f;
    float vertical_kp=3.0; 
    float vertical_kd=0.02;
    float velocity_kp=0.0;
    float velocity_ki=0.001;
    float target_velocity=0;

    float pitch_angvol = 0.0f;
    float RPM = 0.0f;
};

#endif // PID_HPP
