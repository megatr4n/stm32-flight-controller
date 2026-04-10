#include "mpu6050.h"

namespace Devices {

MPU6050::MPU6050(Core::I_I2C* i2c_bus) : i2c(i2c_bus) {}

bool MPU6050::init() {
    uint8_t sensor_id = 0;
    
    if (i2c->readRegister(DEVICE_ADDRESS, REG_WHO_AM_I, &sensor_id, 1)) {
        if (sensor_id == EXPECTED_WHO_AM_I) {
            return true;
        }
    }
    return false;
}

void MPU6050::update() {
}

Core::IMUData MPU6050::getData() const {
    return Core::IMUData{}; 
}

}