#pragma once
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;

void UART2_Init();
void UART_Print(const char* text);