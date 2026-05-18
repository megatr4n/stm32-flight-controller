#include "mpu6050.h"
#include "hal/timing.h"
#include <math.h>

namespace Devices {

MPU6050::MPU6050(Core::I_I2C* i2c_bus) : i2c(i2c_bus) {}

void MPU6050::calculateOffsets() {
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    const int samples = 1000;
    uint8_t rawData[6];

    for (int i = 0; i < samples; i++) {
        if (i2c->readRegister(DEVICE_ADDRESS, REG_GYRO_XOUT_H, rawData, 6)) {
            sumX += (int16_t)((rawData[0] << 8) | rawData[1]);
            sumY += (int16_t)((rawData[2] << 8) | rawData[3]);
            sumZ += (int16_t)((rawData[4] << 8) | rawData[5]);
        }
        HAL_Delay(1);
    }

    gyroOffset.x = ((float)sumX / samples) / GYRO_SCALE;
    gyroOffset.y = ((float)sumY / samples) / GYRO_SCALE;
    gyroOffset.z = ((float)sumZ / samples) / GYRO_SCALE;
}

bool MPU6050::init() {
    uint8_t sensor_id = 0;
    
    if (i2c->readRegister(DEVICE_ADDRESS, REG_WHO_AM_I, &sensor_id, 1)) {
        if (sensor_id == EXPECTED_WHO_AM_I) {
            
            uint8_t reset_cmd = 0x80;
            i2c->writeRegister(DEVICE_ADDRESS, REG_PWR_MGMT_1, &reset_cmd, 1);
            HAL_Delay(100);

            uint8_t pwr_mgmt_data = 0x01;
            if (i2c->writeRegister(DEVICE_ADDRESS, REG_PWR_MGMT_1, &pwr_mgmt_data, 1)) {
                
                calculateOffsets(); 
                
                return true;
            }
        }
    }
    return false;
}

void MPU6050::update() {
    uint8_t rawData[14];

    if (i2c->readRegister(DEVICE_ADDRESS, REG_ACCEL_XOUT_H, rawData, 14)) {
        int16_t accelX = (rawData[0] << 8) | rawData[1];
        int16_t accelY = (rawData[2] << 8) | rawData[3];
        int16_t accelZ = (rawData[4] << 8) | rawData[5];

        int16_t temp = (rawData[6] << 8) | rawData[7];

        int16_t gyroX  = (rawData[8] << 8) | rawData[9];
        int16_t gyroY  = (rawData[10] << 8) | rawData[11];
        int16_t gyroZ  = (rawData[12] << 8) | rawData[13];

        currentData.accel.x = (float)accelX / ACCEL_SCALE;
        currentData.accel.y = (float)accelY / ACCEL_SCALE;
        currentData.accel.z = (float)accelZ / ACCEL_SCALE;

        currentData.gyro.x = ((float)gyroX / GYRO_SCALE) - gyroOffset.x;
        currentData.gyro.y = ((float)gyroY / GYRO_SCALE) - gyroOffset.y;
        currentData.gyro.z = ((float)gyroZ / GYRO_SCALE) - gyroOffset.z;

        currentData.temp = ((float)temp / 340.0f) + 36.53f;

        uint32_t currentTimeUs = HAL::Timing::getMicros();
        float dt = (currentTimeUs - lastUpdateTime) / 1000000.0f;
        lastUpdateTime = currentTimeUs;

        if(dt <= 0.0001f || dt > 0.5f) dt = 0.002f;
        
        float accelPitch = atan2(currentData.accel.y, sqrt(currentData.accel.x * currentData.accel.x + currentData.accel.z * currentData.accel.z)) * RAD_TO_DEG;
        float accelRoll  = atan2(-currentData.accel.x, currentData.accel.z) * RAD_TO_DEG;

        currentData.pitch = 0.98f * (currentData.pitch + currentData.gyro.x * dt) + 0.02f * accelPitch;
        currentData.roll  = 0.98f * (currentData.roll + currentData.gyro.y * dt)  + 0.02f * accelRoll;
    }
}

Core::IMUData MPU6050::getData() const {
    return currentData; 
}

}