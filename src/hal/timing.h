#pragma once
#include "stm32f1xx_hal.h"

namespace HAL {
    class Timing {
    public:
        static void init() {
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            
            DWT->CYCCNT = 0;
            
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        }

        static uint32_t getMicros() {
            return DWT->CYCCNT / (SystemCoreClock / 1000000);
        }
    };
}