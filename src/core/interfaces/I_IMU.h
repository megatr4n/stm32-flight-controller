#pragma once

namespace Core {
    struct Vector3D {
        float x;
        float y;
        float z;
    };

    struct IMUData {
        Vector3D accel;
        Vector3D gyro;
        float temp;
    };

    class I_IMU {
    public:
        virtual ~I_IMU() = default;
        [[nodiscard]] virtual bool init() = 0;
        virtual void update() = 0;
        [[nodiscard]] virtual IMUData getData() = 0;
    };
}