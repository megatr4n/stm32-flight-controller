#pragma once
#include "stm32f1xx_hal.h"

namespace HAL {
    class Timing {
    public:
        static void init() {
            __HAL_RCC_TIM4_CLK_ENABLE();

            htim4.Instance = TIM4;
            htim4.Init.Prescaler = (SystemCoreClock / 1000000) - 1;
            htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
            htim4.Init.Period = 0xFFFF;
            htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
            HAL_TIM_Base_Init(&htim4);

            __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
            __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
            HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(TIM4_IRQn);

            HAL_TIM_Base_Start(&htim4);
        }

        static uint32_t getMicros() {
            __disable_irq();
            uint32_t overflow = overflowCount;
            uint16_t counter = __HAL_TIM_GET_COUNTER(&htim4);
            if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)) {
                overflow++;
                counter = __HAL_TIM_GET_COUNTER(&htim4);
            }
            __enable_irq();
            return (overflow << 16) | counter;
        }

        static void handleIRQ() {
            if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)) {
                __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
                overflowCount++;
            }
        }

    private:
        static TIM_HandleTypeDef htim4;
        static volatile uint32_t overflowCount;
    };
}