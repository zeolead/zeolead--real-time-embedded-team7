#ifndef __DEV_CONFIG_HPP_
#define __DEV_CONFIG_HPP_

#include <stdint.h>
#include "Debug.hpp"
#include <lgpio.h>

/**
 * data
**/
using UBYTE = uint8_t;
using UWORD = uint16_t;
using UDOUBLE = uint32_t;



/*------------------------------------------------------------------------------------------------------*/
class DEV_Config {
public:
    /**
 * GPIO layer
**/
    static constexpr int M1_ENABLE_PIN = 12;
    static constexpr int M1_DIR_PIN = 13;
    static constexpr int M1_STEP_PIN = 19;
    static constexpr int M1_M0_PIN = 16;
    static constexpr int M1_M1_PIN = 17;
    static constexpr int M1_M2_PIN = 20;

    static constexpr int M2_ENABLE_PIN = 4;
    static constexpr int M2_DIR_PIN = 24;
    static constexpr int M2_STEP_PIN = 18;
    static constexpr int M2_M0_PIN = 21;
    static constexpr int M2_M1_PIN = 22;
    static constexpr int M2_M2_PIN = 27;

    static int GPIO_Handle;  //  GPIO ¾ä±ú

    // GPIO Ïà¹Øº¯Êý
    static int DEV_ModuleInit();
    static void DEV_ModuleExit();
    static void DEV_Digital_Write(UWORD Pin, UBYTE Value);

    static void DEV_Delay_ms(UDOUBLE xms);
    static void DEV_Delay_us(UDOUBLE xus);

};
#endif