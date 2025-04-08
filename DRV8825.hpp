#ifndef DRV8825_HPP_
#define DRV8825_HPP_

#include "Debug.hpp"
#include "DEV_Config.hpp"

/*------------------------------------------------------------------------------------------------------*/

class DRV8825 {
public:
    enum MotorID { MOTOR1 = 1, MOTOR2 = 2 };
    enum Direction { FORWARD = 0, BACKWARD = 1 };
    enum ControlMode { HARDWARE = 0, SOFTWARE = 1 };

    struct Motor {
        UBYTE Name;
        const char* MicroStep;
        UBYTE Dir;
        UBYTE EnablePin;
        UBYTE DirPin;
        UBYTE StepPin;
        UBYTE M0Pin;
        UBYTE M1Pin;
        UBYTE M2Pin;
    };

    static void SelectMotor(UBYTE name);
    static void Enable();
    static void Stop();
    static void SetMicroStep(char mode, const char* stepformat);
    static void TurnStep(UBYTE dir, UWORD steps, UWORD stepdelay);
};

#endif // DRV8825_HPP_
