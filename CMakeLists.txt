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
    Webservercontroller.cpp
)

set(HEADER_FILES
    PID.hpp
    mpu6050_Kalman.hpp
    MotorControl.hpp
    DRV8825.hpp
    DEV_Config.hpp
    Debug.hpp	
    Webservercontroller.hpp
)

# 设置 include 目录（可以加多个）
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(PID
    lgpio
    pthread
)