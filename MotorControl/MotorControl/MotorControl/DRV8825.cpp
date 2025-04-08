#include "DRV8825.hpp"
#include "Debug.hpp"    //DEBUG()
#include<algorithm>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <array>

constexpr std::array<const char*, 6> microstepmode = {
    "fullstep", "halfstep", "1/4step", "1/8step", "1/16step", "1/32step"
};
/**
*Example:
*DRV8825::SelectMotor(DRV8825::MOTOR1)
*/

DRV8825::Motor* DRV8825::current_motor_ = nullptr;

void DRV8825::SelectMotor(Motor& motor, UBYTE name) {
    current_motor_ = &motor;
    motor.Name = name;
    if (name == MOTOR1) {
        motor.EnablePin = DEV_Config::M1_ENABLE_PIN;
        motor.DirPin = DEV_Config::M1_DIR_PIN;
        motor.StepPin = DEV_Config::M1_STEP_PIN;
        motor.M0Pin = DEV_Config::M1_M0_PIN;
        motor.M1Pin = DEV_Config::M1_M1_PIN;
        motor.M2Pin = DEV_Config::M1_M2_PIN;
    }
    else if (name == MOTOR2) {
        motor.EnablePin = DEV_Config::M2_ENABLE_PIN;
        motor.DirPin = DEV_Config::M2_DIR_PIN;
        motor.StepPin = DEV_Config::M2_STEP_PIN;
        motor.M0Pin = DEV_Config::M2_M0_PIN;
        motor.M1Pin = DEV_Config::M2_M1_PIN;
        motor.M2Pin = DEV_Config::M2_M2_PIN;
    }
    else {
        Debug::Log("please set motor: MOTOR1 or MOTOR2\n");
    }
}

/**
 * The motor stops rotating and the driver chip is disabled.
 *
 */
void DRV8825::Enable() {
    Debug::Log("Enable() called for motor %d\n", current_motor_->Name);
    DEV_Config::DEV_Digital_Write(current_motor_->EnablePin, 1);
}

void DRV8825::Stop() {
    Debug::Log("Stop() called for motor %d\n", current_motor_ -> Name);
    DEV_Config::DEV_Digital_Write(current_motor_ -> EnablePin, 0);
}

void DRV8825::SetMicroStep(Motor& motor, char mode, const char* stepformat) {
    if (mode == HARDWARE) {
        Debug::Log("use hardware control\n");
        return;
    }
    Debug::Log("use software control\n");
    Debug::Log("step format = %s\n", stepformat);

    auto it = std::find(microstepmode.begin(), microstepmode.end(), stepformat);
    if (it != microstepmode.end()) {
        motor.MicroStep = *it;
        int index = std::distance(microstepmode.begin(), it);
        DEV_Config::DEV_Digital_Write(motor.M0Pin, index & 0x01);
        DEV_Config::DEV_Digital_Write(motor.M1Pin, (index >> 1) & 0x01);
        DEV_Config::DEV_Digital_Write(motor.M2Pin, (index >> 2) & 0x01);
    }
    else {
        Debug::Log("Invalid step format!\n");
        exit(0);
    }
}

void DRV8825::TurnStep(Motor& motor, UBYTE dir, UWORD steps, UWORD stepdelay) {
    motor.Dir = dir;
    if (dir == FORWARD) {
        Debug::Log("motor %d forward\n", motor.Name);
        DEV_Config::DEV_Digital_Write(motor.DirPin, 0);
    }
    else if (dir == BACKWARD) {
        Debug::Log("motor %d backward\n", motor.Name);
        DEV_Config::DEV_Digital_Write(motor.DirPin, 1);
    }
    else {
        Debug::Log("Invalid direction: %d, calling Stop()\n", dir);
        Stop();
        return;
    }

    if (steps == 0) return;

    Debug::Log("turn delay %d \n", stepdelay);
    for (UWORD i = 0; i < steps; i++) {
        DEV_Config::DEV_Digital_Write(motor.StepPin, 1);
        DEV_Config::DEV_Delay_ms(stepdelay);
        DEV_Config::DEV_Digital_Write(motor.StepPin, 0);
        DEV_Config::DEV_Delay_ms(stepdelay);
    }
}
