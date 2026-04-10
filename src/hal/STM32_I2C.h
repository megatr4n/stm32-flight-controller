#pragma once
#include "../core/interfaces/I_I2C.h"
#include "stm32f1xx_hal.h"

namespace HAL {
    class STM32_I2C : public Core::I_I2C {
    private:
        I2C_HandleTypeDef hi2c1;

    public:
        STM32_I2C();

        [[nodiscard]] bool init() override;
        [[nodiscard]] bool readRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint16_t length) override;
        [[nodiscard]] bool writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint16_t length) override;
    };
}