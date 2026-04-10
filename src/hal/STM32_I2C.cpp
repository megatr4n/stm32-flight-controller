#include "STM32_I2C.h"

namespace HAL {
    STM32_I2C::STM32_I2C() {

    }

    bool STM32_I2C::init() {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        hi2c1.Instance = I2C1;
        hi2c1.Init.ClockSpeed = 100000;
        hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
        hi2c1.Init.OwnAddress1 = 0;
        hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        hi2c1.Init.OwnAddress2 = 0;
        hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

        if (HAL_I2C_Init(&hi2c1) == HAL_OK) {
            return true;
        }
        return false;
    }
    bool STM32_I2C::readRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint16_t length) {
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, deviceAddress, registerAddress, 1, data, length, 100);
        return (status == HAL_OK);
    }

    bool STM32_I2C::writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint16_t length) {
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, deviceAddress, registerAddress, 1, data, length, 100);
        return (status == HAL_OK);
    }

}