WebSocket Segway
======
![image](https://github.com/zeolead/zeolead--real-time-embedded-team7/blob/main/WebSocket%20Segway.jpg)

Studying the dynamic balance of a vehicle on two wheels is essential for future research on the dynamic stability of robots and forms the foundation for understanding and mastering complex robotics technologies.

Current situation: All system works well. This segway could not develop further unless it has a independent power source.
## File Introduction

Below is an overview of the main source files in this project:

- **`main.cpp`**: Program entry point. Handles module initialization, signal handling, launching the MPU6050 sensor thread, PID control thread, WebSocket server thread, and motor control threads.
- **`PID.hpp` / `PID.cpp`**: PID control algorithm implementation. Contains the tilt-angle loop (`Vertical`) and velocity loop (`Velocity`), integrates Kalman-filtered IMU input, velocity estimation, and supports dynamic parameter updates.
- **`mpu6050_Kalman.hpp` / `mpu6050_Kalman.cpp`**: IMU data acquisition and Kalman filter fusion for the MPU6050 sensor. Implements I2C communication, low-pass filtering, bias calibration, and real-time orientation estimation.
- **`MotorControl.hpp` / `MotorControl.cpp`**: Thread-safe stepper motor control wrapper. Provides RPM setting, direction control, step pulse generation, and status/step callback interfaces.
- **`DRV8825.hpp` / `DRV8825.cpp`**: Abstraction for the DRV8825 stepper motor driver. Configures microstepping, enable/disable, direction, and step pulses using `DEV_Config` for low-level GPIO operations.
- **`DEV_Config.hpp` / `Dev_Config.cpp`**: Raspberry Pi GPIO initialization and control utilities. Detects board version, claims GPIO pins, and provides millisecond/microsecond delay functions.
- **`Webservercontroller.hpp` / `Webservercontroller.cpp`**: Remote control module based on WebSocket++. Handles server setup, incoming message callbacks, and integration with the main control logic.
- **`CMakeLists.txt`**: Build script. Specifies C++ standard, executable targets, source files, and links against `pthread` and `lgpio` libraries.
- **`Debug.hpp`**: Debug logging utility. Offers optional printouts for runtime diagnostics across modules.
## Key Features
>* Real-time Tilt Angle Monitoring
>* PID Control Algorithm
>* WebSocket-Based Remot Contnrol
## Technical Challenges
>* Low Latency and Fast Response
>* Motor Control and Power Distribution
>* System Integration and Debugging
>* Network Communication and Command Parsing
## Hardware Structure Design
![image](https://github.com/zeolead/zeolead--real-time-embedded-team7/blob/main/Hardware%20Structure%20Design.jpg)
## Software Structure Design
![image](https://github.com/zeolead/zeolead--real-time-embedded-team7/blob/main/Software%20Structure%20Design.jpg)

## Project Progress Log

### February 27, 2025 (Thursday)
- Successfully flashed the Raspberry Pi OS.
- All team members can remotely access the Pi from our devices.
- All ordered hardware components have arrived.

### March 12, 2025 (Wednesday)
- Setting up Bluetooth structure on Windows.

### March 13, 2025 (Thursday)
- All components have been soldered.
- Hardware connections are complete. Now moving on to software development and debugging.

### March 18, 2025 (Tuesday)
- 3D printing of the chassis completed; physical structure is assembled.
- Sensor calibration completed; IMU is functioning properly.
- Initial motor control implemented; motor response testing has begun.
- Basic implementation of the PID control algorithm completed.

### March 19, 2025 (Wednesday)
- Changed to Websocket connection for only JavaScript development is allowed on the PC side.

### March 23, 2025 (Sunday)
- Completed the design of the web interface.

### April 1, 2025 (Tuesday)
- Chassis assembly completed; all modules are mounted.
- Project enters debugging phase in preparation for the first autonomous run test.

### April 6, 2025 (Sunday)
- Finished writing the backend code for the web interface.

### April 7, 2025 (Monday)
- Significant mismatch between sensor tilt angle data and actual motion.
- Motor failed to start.

### April 8, 2025 (Tuesday)
- Motor now starts correctly after adding initialization logic; enters loop waiting for control commands.
- Optimized all control processes; initial operational pipeline established.

### April 9, 2025 (Wednesday)
- Motor exhibited "speed overflow" issue. Preliminary diagnosis suggests virtual speed value keeps accumulating, possibly due to sensor reading errors or the lack of encoders.

### April 10, 2025 (Thursday)
- Sensor axes were flipped during mounting; adjusted coordinate system in code to correct angle output errors.
- Rewrote both speed and balance control loops; successfully resolved speed overflow issue.

### April 12, 2025 (Saturday)
- Testing moved to the lab — wheels mounted, and all systems are go for integrated debugging.

### April 13, 2025 (Sunday)
- Retuned complementary filter to improve angle accuracy.
- Discovered sensor output delay and sluggish response.
- The robot can stand upright but shows oscillation at rest, and no forward movement occurs.
- Connected to the server on Raspberry PI.

### April 14, 2025 (Monday)
- Replaced complementary filter with Kalman filter to fix data latency.
- Solved oscillation at rest by setting a dead zone and using theoretical speed instead of encoder feedback.
- Fine-tuned PID parameters to improve motor responsiveness.
- Adjusted pulse frequency to avoid jitter or missed steps, aiming to find the maximum stable speed.

### April 15, 2025 (Tuesday)
- Implemented real-time visualization of sensor data (acceleration-based angle vs actual angle), aiding in validating the attitude estimation algorithm and parameter tuning.

### April 17, 2025 (Thursday)
- Attempted to upgrade power supply using a power bank; voltage found to be insufficient.

### April 18, 2025 (Friday)
- Tried using a boost converter with the power bank, but encountered instability. Reverted to the original stable power socket setup.

### April 19, 2025 (Saturday)
- Designed and implemented the server-side threading model.

## Want to see more?
We share extra project updates, behind-the-scenes content, and little surprises on Instagram —— don't miss out!

Follow us: @2990788f

---

