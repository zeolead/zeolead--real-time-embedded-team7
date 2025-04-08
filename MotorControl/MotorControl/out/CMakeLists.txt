# CMakeList.txt: MotorControl 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#

cmake_minimum_required(VERSION 3.10)

project(motorControl)

# 将源代码添加到此项目的可执行文件。
add_executable (MotorControl "MotorControl.cpp" 
"MotorControl.hpp"
"Dev_Config.cpp"
"DEV_Config.hpp"
"DRV8825.cpp"
"DRV8825.hpp"
"Debug.hpp"
"motor_main.cpp")

target_link_libraries(MotorControl lgpio pthread)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET MotorControl PROPERTY CXX_STANDARD 20)
endif()

# TODO: 如有需要，请添加测试并安装目标。
