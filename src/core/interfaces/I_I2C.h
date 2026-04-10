#pragma once
#include <stdint.h>

namespace Core {

class I_I2C {
public:
    virtual ~I_I2C() = default;

    [[nodiscard]] virtual bool init() = 0;
    [[nodiscard]] virtual bool readRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint16_t length) = 0;
    [[nodiscard]] virtual bool writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint16_t length) = 0;
};

}