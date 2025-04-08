cmake_minimum_required(VERSION 3.10)
project(motorControl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置源文件
set(SOURCES
    motor_main.cpp
    MotorControl.cpp
    Dev_Config.cpp
    DRV8825.cpp
)

# 设置 include 目录（可以加多个）
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 构建可执行文件
add_executable(MotorControl ${SOURCES})

# 链接外部库
target_link_libraries(MotorControl lgpio pthread)
