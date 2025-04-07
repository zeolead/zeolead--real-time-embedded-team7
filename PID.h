#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

// PID  controller
float Vertical(float Expect, float Angle, float gyro_x);
float Velocity(float angle_pid_output);

// Motor control
void driveMotorWithVelocityOutput(float motor_speed_output);

// PID Coefficent
extern float vertical_kp, vertical_ki, vertical_kd;
extern float velocity_kp, velocity_ki, velocity_kd;
extern float turn_kp, turn_ki, turn_kd;

#ifdef __cplusplus
}
#endif

#endif
