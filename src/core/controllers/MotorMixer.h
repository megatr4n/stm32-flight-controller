#pragma once
#include <stdint.h>

namespace Core {
    
    struct MotorSpeeds {
        uint16_t frontLeft;
        uint16_t frontRight;
        uint16_t rearLeft;
        uint16_t rearRight;
    };

    class MotorMixer {
        private:
        static uint16_t constrain(float val) {
            if (val < 1000.0f) {
                return 1000;
            }
            if (val > 2000.0f) {
                return 2000;
            }
            return (uint16_t)val; 
        }

    public:
        static MotorSpeeds mix(uint16_t baseThrottle, float pidPitch, float pidRoll, float pidYaw) {
            MotorSpeeds speeds;

            float fl = baseThrottle + pidPitch + pidRoll - pidYaw;
            float fr = baseThrottle + pidPitch - pidRoll + pidYaw;
            float rl = baseThrottle - pidPitch + pidRoll + pidYaw;
            float rr = baseThrottle - pidPitch - pidRoll - pidYaw;

            speeds.frontLeft = constrain(fl);
            speeds.frontRight = constrain(fr);
            speeds.rearLeft = constrain(rl);
            speeds.rearRight = constrain(rr);

            return speeds;
        }
    };
}