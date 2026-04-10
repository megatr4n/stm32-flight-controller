#pragma once
#include "../core/interfaces/I_IMU.h"
#include "../core/interfaces/I_I2C.h"
#include <stdint.h>

namespace Devices {

class MPU6050 : public Core::I_IMU {
private:
    Core::I_I2C* i2c; 

    static const uint8_t DEVICE_ADDRESS = 0xD0;
    static const uint8_t REG_WHO_AM_I = 0x75;
    static const uint8_t EXPECTED_WHO_AM_I = 0x68;

public:
    explicit MPU6050(Core::I_I2C* i2c_bus);

    [[nodiscard]] bool init() override;
    void update() override;
    [[nodiscard]] Core::IMUData getData() const override; 
};

}