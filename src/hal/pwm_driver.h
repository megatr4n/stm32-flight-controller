#pragma once
#include <stdint.h>

namespace HAL {
    class PWMDriver {
    public:
        bool init();
        void setMotorSpeeds(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
    };
}