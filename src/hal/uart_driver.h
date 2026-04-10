#pragma once
#include "stm32f1xx_hal.h"

namespace HAL {

extern UART_HandleTypeDef huart2;

void UART2_Init(void);
void UART_Print(const char* text);

}