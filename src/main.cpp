#include "stm32f1xx_hal.h"
#include "hal/STM32_I2C.h"
#include "hal/uart_driver.h"
#include "devices/mpu6050.h"
#include <stdio.h>

using namespace HAL;
using namespace Devices;

static void SystemClock_Config(void)
{
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

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    UART1_Init();
    UART_Print("\r\nFlight Controller Started\r\n");

    STM32_I2C i2cBus;
    if (!i2cBus.init())
    {
        UART_Print("CRITICAL ERROR: I2C Bus failed to start!\r\n");
        while (1)
            ;
    }

    MPU6050 gyro(&i2cBus);

    if (gyro.init())
    {
        UART_Print("MPU6050 Initialized and Woken Up!\r\n");

        char buffer[128];

        while (1)
        {
            gyro.update();

            Core::IMUData data = gyro.getData();

            snprintf(buffer, sizeof(buffer), "Pitch: %d deg | Roll: %d deg\r\n",
                     (int)data.pitch,
                     (int)data.roll);

            UART_Print(buffer);

            HAL_Delay(20);
        }
    }
    else
    {
        UART_Print("Error! MPU6050 Not Found!\r\n");
        while (1)
            ;
    }
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}