#include "DEV_Config.hpp"
#include "Debug.hpp"
#include <cstdio>

#include <lgpio.h>

int DEV_Config::GPIO_Handle = -1;

int DEV_Config::DEV_ModuleInit() {
    char buffer[4];
    FILE* fp = popen("cat /proc/cpuinfo | grep 'Raspberry Pi 5'", "r");
    if (!fp) {
        Debug::Log("Cannot determine Raspberry Pi model\n");
        return -1;
    }

    if (fgets(buffer, sizeof(buffer), fp) != nullptr) {
        GPIO_Handle = lgGpiochipOpen(4);
    }
    else {
        GPIO_Handle = lgGpiochipOpen(0);
    }
    pclose(fp);

    if (GPIO_Handle < 0) {
        Debug::Log("Failed to open GPIO chip\n");
        return -1;
    }

    lgGpioClaimOutput(GPIO_Handle, 0, M1_ENABLE_PIN, LG_LOW);
    lgGpioClaimOutput(GPIO_Handle, 0, M1_DIR_PIN, LG_LOW);
    lgGpioClaimOutput(GPIO_Handle, 0, M1_STEP_PIN, LG_LOW);
    lgGpioClaimOutput(GPIO_Handle, 0, M1_M0_PIN, LG_LOW);
    lgGpioClaimOutput(GPIO_Handle, 0, M1_M1_PIN, LG_LOW);
    lgGpioClaimOutput(GPIO_Handle, 0, M1_M2_PIN, LG_LOW);

    lgGpioClaimOutput(GPIO_Handle, 0, M2_ENABLE_PIN, LG_LOW);
    lgGpioClaimOutput(GPIO_Handle, 0, M2_DIR_PIN, LG_LOW);
    lgGpioClaimOutput(GPIO_Handle, 0, M2_STEP_PIN, LG_LOW);
    lgGpioClaimOutput(GPIO_Handle, 0, M2_M0_PIN, LG_LOW);
    lgGpioClaimOutput(GPIO_Handle, 0, M2_M1_PIN, LG_LOW);
    lgGpioClaimOutput(GPIO_Handle, 0, M2_M2_PIN, LG_LOW);

    return 0;
}

void DEV_Config::DEV_ModuleExit() {
    if (GPIO_Handle >= 0) {
        lgGpiochipClose(GPIO_Handle);
    }
}

void DEV_Config::DEV_Digital_Write(UWORD Pin, UBYTE Value) {
    lgGpioWrite(GPIO_Handle, Pin, Value);
}

void DEV_Config::DEV_Delay_ms(UDOUBLE xms) {
    lguSleep(xms / 1000000.0);
}

void DEV_Config::DEV_Delay_us(UDOUBLE xus) {
    for (volatile int j = xus; j > 0; --j);
}
