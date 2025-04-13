#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include <cstdio>

class Debug {
public:
    static constexpr bool USE_DEBUG = true;

    template<typename... Args>
    static void Log(const char* format, Args... args) {
        if constexpr (USE_DEBUG) {
            printf("Debug: ");
            printf(format, args...);
        }
    }
};

#endif // DEBUG_HPP_
