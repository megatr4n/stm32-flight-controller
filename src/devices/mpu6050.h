#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

class MPU6050 {
private:
    static const uint8_t DEVICE_ADDRESS = 0xD0;
    static const uint8_t REG_WHO_AM_I = 0x75;
    static const uint8_t EXPECTED_WHO_AM_I = 0x68;

public:
    MPU6050();
    bool CheckConnection();
};