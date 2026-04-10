#pragma once
#include "stm32f1xx_hal.h"

namespace HAL {

extern UART_HandleTypeDef huart1;

void UART1_Init(void);
void UART_Print(const char* text);

}