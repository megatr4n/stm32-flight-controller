#pragma once

namespace Core {

    struct PIDConfig {
        float kp;
        float ki;
        float kd;
        float minOutput;
        float maxOutput;
    };

    class PIDController {
    private:
        PIDConfig config;
        float integral = 0;
        float prevError = 0;

    public:
        explicit PIDController(const PIDConfig& cfg) : config(cfg) {}
        float calculate(float setpoint, float currentMeasuredValue, float dt) {
            float error = setpoint - currentMeasuredValue;
            float pTerm = config.kp * error;

            integral += error * dt;
            float iTerm = config.ki * integral;

            float derivative = (error - prevError) / dt;
            float dTerm = config.kd * derivative;

            prevError = error;

            float output = pTerm + iTerm + dTerm;
            if (output > config.maxOutput)
                output = config.maxOutput;

            if (output < config.minOutput)
                output = config.minOutput;

            return output;
        }
        void reset() {
            integral = 0;
            prevError = 0;
        }
    };
}