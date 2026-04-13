#include "stm32f1xx_hal.h"
#include "hal/STM32_I2C.h"
#include "hal/uart_driver.h"
#include "devices/mpu6050.h"

#include "core/controllers/PIDController.h"
#include "core/controllers/MotorMixer.h"

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

int main(void) {
    HAL_Init();
    SystemClock_Config();
    UART1_Init();
    
    UART_Print("\r\nFlight Controller Starting!\r\n");

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

    PIDConfig pitchConfig = {2.0f, 0.0f, 0.5f, -400.0f, 400.0f};
    PIDConfig rollConfig  = {2.0f, 0.0f, 0.5f, -400.0f, 400.0f};

    PIDController pidPitch(pitchConfig);
    PIDController pidRoll(rollConfig);

    char buffer[128];
    uint32_t lastLoopTime = HAL_GetTick();

    UART_Print("Starting Flight Loop...\r\n");

    while (1) {
        uint32_t currentTime = HAL_GetTick();
        float dt = (currentTime - lastLoopTime) / 1000.0f;
        lastLoopTime = currentTime;
        
        if (dt <= 0.001f) dt = 0.001f; 

        gyro.update();
        IMUData data = gyro.getData();

        float pitchCorrection = pidPitch.calculate(0.0f, data.pitch, dt);
        float rollCorrection  = pidRoll.calculate(0.0f, data.roll, dt);

        uint16_t baseThrottle = 1300; 
        MotorSpeeds speeds = MotorMixer::mix(baseThrottle, pitchCorrection, rollCorrection);

        snprintf(buffer, sizeof(buffer), 
                 "P: %3d R: %3d || M1: %d | M2: %d | M3: %d | M4: %d\r\n",
                 (int)data.pitch, (int)data.roll,
                 speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight);
        
        UART_Print(buffer);
        HAL_Delay(20); 
    }
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}