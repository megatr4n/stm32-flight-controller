#pragma once

#include "stm32f1xx_hal.h"
#include "devices/mpu6050.h"
#include "hal/pwm_driver.h"
#include "hal/IBusReceiver.h"
#include "hal/timing.h"
#include "hal/uart_driver.h"

#include "core/controllers/PIDController.h"
#include "core/controllers/MotorMixer.h"
#include "core/filters/LowPassFilter.h"
#include "core/controllers/SystemState.h"
#include "core/controllers/CLI.h"

namespace Core {

    class FlightTask {
    private:
        Devices::MPU6050* gyro;
        HAL::PWMDriver* pwm;
        HAL::IBusReceiver* receiver;

        SystemStateMachine stateMachine;
        PIDController pidPitch;
        PIDController pidRoll;
        PIDController pidYaw;
        LowPassFilter filterPitch;
        LowPassFilter filterRoll;
        LowPassFilter filterYaw;
        CommandParser cli;

        uint32_t lastLoopTimeUs;
        uint32_t lastPrintTimeMs;

        static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }

    public:
        FlightTask(Devices::MPU6050* g, HAL::PWMDriver* p, HAL::IBusReceiver* r)
            : gyro(g), pwm(p), receiver(r),
              pidPitch({2.0f, 0.0f, 0.5f, -400.0f, 400.0f, 100.0f}),
              pidRoll({2.0f, 0.0f, 0.5f, -400.0f, 400.0f, 100.0f}),
              pidYaw({3.0f, 0.0f, 0.0f, -400.0f, 400.0f, 100.0f}),
              filterPitch(0.15f), filterRoll(0.15f), filterYaw(0.15f),
              cli(&pidPitch, &pidRoll, &pidYaw) 
        {
            lastLoopTimeUs = 0;
            lastPrintTimeMs = 0;
        }

        void feedCLI(char c) {
            cli.feedChar(c);
        }

        void start(bool calibrationRequested) {
            stateMachine.notifyInitComplete(calibrationRequested);
            lastLoopTimeUs = HAL::Timing::getMicros();
            lastPrintTimeMs = HAL_GetTick();
            HAL::UART_Print("Starting Flight Task...\r\n");
        }

        void update() {
            uint32_t currentTimeUs = HAL::Timing::getMicros();
            
            if (currentTimeUs - lastLoopTimeUs >= 2000) {
                float dt = (currentTimeUs - lastLoopTimeUs) / 1000000.0f;
                lastLoopTimeUs = currentTimeUs; 
                
                if (dt <= 0.001f || dt > 0.05f) dt = 0.002f; 

                Core::ReceiverData rcData = receiver->getRCData();
                bool is_connected = receiver->isConnected();

                #ifdef WOKWI_SIMULATION
                    is_connected = true;       
                    rcData.aux1 = 2000;       
                    rcData.pitch = 1500;      
                    rcData.roll = 1500;
                    rcData.yaw = 1500;
                    
                    static bool isSimArmed = false;
                    if (stateMachine.areMotorsAllowed()) isSimArmed = true;
                    rcData.throttle = isSimArmed ? 1300 : 1000;
                #endif

                bool armSwitch = (rcData.aux1 > 1500);
                stateMachine.update(is_connected, armSwitch, rcData.throttle);

                gyro->update();
                IMUData data = gyro->getData();

                float cleanPitch = filterPitch.apply(data.pitch);
                float cleanRoll  = filterRoll.apply(data.roll);
                float cleanYaw   = filterYaw.apply(data.gyro.z);

                float targetPitch = mapFloat(rcData.pitch, 1000.0f, 2000.0f, -30.0f, 30.0f);
                float targetRoll  = mapFloat(rcData.roll,  1000.0f, 2000.0f, -30.0f, 30.0f);
                float targetYaw   = mapFloat(rcData.yaw,   1000.0f, 2000.0f, -90.0f, 90.0f);

                float pitchCorrection = pidPitch.calculate(targetPitch, cleanPitch, dt);
                float rollCorrection  = pidRoll.calculate(targetRoll, cleanRoll, dt);
                float yawCorrection   = pidYaw.calculate(targetYaw, cleanYaw, dt);

                uint16_t baseThrottle = 1000;
                MotorSpeeds speeds = {1000, 1000, 1000, 1000};
                
                if (stateMachine.areMotorsAllowed()) {
                    baseThrottle = rcData.throttle;
                    speeds = MotorMixer::mix(baseThrottle, pitchCorrection, rollCorrection, yawCorrection);
                } 
                else if (stateMachine.isCalibratingESC()) {
                    speeds.frontLeft  = rcData.throttle;
                    speeds.frontRight = rcData.throttle;
                    speeds.rearLeft   = rcData.throttle;
                    speeds.rearRight  = rcData.throttle;
                } 
                else {
                    pidPitch.reset();
                    pidRoll.reset();
                    pidYaw.reset();
                    filterPitch.reset();
                    filterRoll.reset();
                    filterYaw.reset();
                }

                pwm->setMotorSpeeds(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight);

                uint32_t currentTimeMs = HAL_GetTick();
                if (currentTimeMs - lastPrintTimeMs >= 1000) {
                    char msg[140];
                    sprintf(msg, "STATE: %d | Thr: %d | Pit: %d || M1: %d | M3: %d\r\n", 
                            static_cast<int>(stateMachine.getState()), rcData.throttle, (int)cleanPitch, speeds.frontLeft, speeds.rearLeft);
                    HAL::UART_Print(msg);
                    lastPrintTimeMs = currentTimeMs; 
                }
            }
        }
    };
}