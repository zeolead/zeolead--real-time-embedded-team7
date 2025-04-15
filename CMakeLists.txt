cmake_minimum_required(VERSION 3.10)
project(PID)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置源文件
set(SOURCES
    main.cpp
    PID.cpp
    mpu6050_test001.cpp
    MotorControl.cpp
    DRV8825.cpp
    Dev_Config.cpp
)

set(HEADER_FILES
    PID.hpp
    mpu6050_test001.hpp
    MotorControl.hpp
    DRV8825.hpp
    DEV_Config.hpp
    Debug.hpp	
)
# 设置 include 目录（可以加多个）
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 构建可执行文件
add_executable(PID ${SOURCES} ${HEADER_FILES})

# 链接外部库
target_link_libraries( PID lgpio pthread)
