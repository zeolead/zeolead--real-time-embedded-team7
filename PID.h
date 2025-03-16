#ifndef PID_H
#define PID_H


int Vertical(float Expect, float Angle, float gyro_x);
float Velocity(float Expect_velocity, float velocityL, float velocityR);


// int Turn(float gyro_z, int Target);

#endif