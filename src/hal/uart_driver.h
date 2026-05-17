#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

namespace HAL {

    extern UART_HandleTypeDef huart1;
    extern DMA_HandleTypeDef hdma_usart1_rx;

    void UART1_Init(void);
    void UART_Print(const char* text);
    
    void UART1_Start_DMA_RX(uint8_t* buffer, uint16_t size); 

}