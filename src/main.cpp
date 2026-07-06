#define WOKWI_SIMULATION 1

#include "stm32f1xx_hal.h"
#include "hal/STM32_I2C.h"
#include "hal/uart_driver.h"
#include "devices/mpu6050.h"
#include "hal/pwm_driver.h"
#include "hal/IBusReceiver.h"
#include "hal/timing.h"

#include "core/controllers/FlightTask.h"

#include <stdio.h>

using namespace HAL;
using namespace Devices;
using namespace Core;

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

uint8_t rx_buffer[64];
uint16_t rx_tail = 0;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    HAL::Timing::init();
    UART1_Init();
    
    UART_Print("\r\nFlight Controller Starting (Clean Arch)!\r\n");

    STM32_I2C i2cBus;
    if (!i2cBus.init()) {
        UART_Print("ERROR: I2C Failed!\r\n");
        while (1);
    }

    MPU6050 gyro(&i2cBus);
    if (!gyro.init()) {
        UART_Print("ERROR: MPU6050 Not Found!\r\n");
        while (1);
    }
    UART_Print("Sensors OK!\r\n");

    PWMDriver pwm;
    if (!pwm.init()) {
        UART_Print("ERROR: PWM Timers Failed!\r\n");
        while(1);
    }
    UART_Print("PWM Timers Initialized!\r\n");

    HAL::IBusReceiver receiver;
    receiver.init();

    UART1_Start_DMA_RX(rx_buffer, 64);
    UART_Print("Receiver Initialized!\r\n");

    Core::FlightTask flightTask(&gyro, &pwm, &receiver);
    
    bool calibrationRequested = false;
    flightTask.start(calibrationRequested);

    while (1) {
        
        #ifdef WOKWI_SIMULATION
            while (USART1->SR & USART_SR_RXNE) {
                uint8_t incomingByte = USART1->DR;
                
                char echo[2] = { (char)incomingByte, '\0' };
                UART_Print(echo);
                
                receiver.feedByte(incomingByte);
                flightTask.feedCLI((char)incomingByte);
            }
        #else
            uint16_t cndtr = __HAL_DMA_GET_COUNTER(&HAL::hdma_usart1_rx);
            uint16_t rx_head = (64 - cndtr) % 64; 
            
            while (rx_tail != rx_head) {
                uint8_t incomingByte = rx_buffer[rx_tail];
                
                receiver.feedByte(incomingByte);
                flightTask.feedCLI((char)incomingByte); 
                
                rx_tail = (rx_tail + 1) % 64;
            }
        #endif

        flightTask.update();
    }
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}