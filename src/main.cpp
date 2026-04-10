#include "stm32f4xx_hal.h"
#include "drivers/i2c_driver.h"
#include "drivers/uart_driver.h"
#include "sensors/mpu6050.h"

int main(void) {
    HAL_Init();
    I2C1_Init();

    UART_Print ("\r\n FLight Controller Starting Launched!\r\n");

    MPU6050 gyro;

    while (1) {
        if (gyro.CheckConnection()) {
            UART_Print("MPU6050 Found! Connection OK!\r\n");
        } else {
            UART_Print("Error! MPU6050 Not Found!");
        }
        HAL_Delay(1000);
    }
}

extern "C" void SysTick_Handler(void) {
    HAL_IncTick();
}