#include "stm32f1xx_hal.h"
#include "hal/STM32_I2C.h"
#include "hal/uart_driver.h"
#include "devices/mpu6050.h"

using namespace HAL;
using namespace Devices;

int main(void) {
    HAL_Init();
    
    UART2_Init();

    UART_Print("\r\nFlight Controller Started\r\n");

    STM32_I2C i2cBus;
    if (!i2cBus.init()) {
        UART_Print("CRITICAL ERROR: I2C Bus failed to start!\r\n");
        while(1); 
    }

    MPU6050 gyro(&i2cBus);

    while (1) {
        if (gyro.init()) {
            UART_Print("MPU6050 Found! Connection OK!\r\n");
        } else {
            UART_Print("Error! MPU6050 Not Found!\r\n");
        }
        
        HAL_Delay(1000);
    }
}

extern "C" void SysTick_Handler(void) {
    HAL_IncTick();
}