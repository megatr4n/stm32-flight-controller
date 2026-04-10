#include "mpu6050.h"
#include "../hal/i2c_driver.h"

MPU6050::MPU6050() {

}

bool MPU6050::CheckConnection() {
    uint8_t sensor_id = 0;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADDRESS, REG_WHO_AM_I, 1, &sensor_id, 1, 1000);

    if (status == HAL_OK && sensor_id == EXPECTED_WHO_AM_I) {
        return true;
    }
    return false;
}