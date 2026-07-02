#include "timing.h"

namespace HAL {
    TIM_HandleTypeDef Timing::htim4;
    volatile uint32_t Timing::overflowCount = 0;
}

extern "C" void TIM4_IRQHandler(void) {
    HAL::Timing::handleIRQ();
}