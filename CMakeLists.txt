cmake_minimum_required(VERSION 3.10)
project(PID)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# —— 主程序 executable ——
add_executable(PID
    main.cpp
    PID.cpp
    mpu6050_Kalman.cpp
    MotorControl.cpp
    DRV8825.cpp
    Dev_Config.cpp
)

set(HEADER_FILES
    PID.hpp
    mpu6050_Kalman.hpp
    MotorControl.hpp
    DRV8825.hpp
    DEV_Config.hpp
    Debug.hpp	
)

# 设置 include 目录（可以加多个）
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(PID
    lgpio
    pthread
)

# —— MPU6050 回调测试 —— (callback_test)
add_executable(callback_test
    callback_test.cpp
    mpu6050_Kalman.cpp
    Dev_Config.cpp        # 如果 MPU6050 里用到 DEV_Config，也需要链接
)

target_link_libraries(callback_test
    lgpio
    pthread
)

# —— PID 输出回调测试 —— (pid_callback_test)
add_executable(pid_callback_test
    pid_callback_test.cpp
    PID.cpp
)

target_link_libraries(pid_callback_test
    pthread
)

# —— MotorControl 回调测试 —— (motorcontrol_callback_test)
add_executable(motorcontrol_callback_test
    motorcontrol_callback_test.cpp
    MotorControl.cpp
    DRV8825.cpp
    Dev_Config.cpp
)

target_link_libraries(motorcontrol_callback_test
    lgpio
    pthread
)
