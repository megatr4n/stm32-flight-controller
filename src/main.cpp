#include "stm32f1xx_hal.h"
#include "hal/STM32_I2C.h"
#include "hal/uart_driver.h"
#include "devices/mpu6050.h"

#include "core/controllers/PIDController.h"
#include "core/controllers/MotorMixer.h"

#include "hal/pwm_driver.h"
#include "hal/IBusReceiver.h"

#include <math.h>
#include <stdio.h>

using namespace HAL;
using namespace Devices;
using namespace Core;

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void simulateIBUS(HAL::IBusReceiver& rx, uint16_t throttle, uint16_t pitch, uint16_t aux1) {
    uint8_t packet[32] = {0};
    packet[0] = 0x20; 
    packet[1] = 0x40; 

    packet[2] = 1500 & 0xFF; 
    packet[3] = (1500 >> 8) & 0xFF;
    packet[4] = pitch & 0xFF; 
    packet[5] = (pitch >> 8) & 0xFF;
    packet[6] = throttle & 0xFF; 
    packet[7] = (throttle >> 8) & 0xFF;
    packet[8] = 1500 & 0xFF; 
    packet[9] = (1500 >> 8) & 0xFF;

    packet[10] = aux1 & 0xFF; 
    packet[11] = (aux1 >> 8) & 0xFF;

    for(int i = 12; i < 30; i += 2) {
        packet[i] = 1500 & 0xFF;
        packet[i+1] = (1500 >> 8) & 0xFF;
    }
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < 30; i++) {
        checksum -= packet[i];
    }
    packet[30] = checksum & 0xFF;
    packet[31] = (checksum >> 8) & 0xFF;
    for (int i = 0; i < 32; i++) {
        rx.feedByte(packet[i]);
    }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    UART1_Init();
    
    UART_Print("\r\nFlight Controller Starting!\r\n");

    STM32_I2C i2cBus;
    if (!i2cBus.init()) {
        UART_Print("ERROR: I2C Failed!\r\n");
        while (1);
    }

    MPU6050 gyro(&i2cBus);
    if (!gyro.init()) {
        UART_Print("ERROR: MPU6050 Not Found!\r\n");
        while (1);
    }
    UART_Print("Sensors OK!\r\n");

    PWMDriver pwm;
    if (!pwm.init()) {
        UART_Print("ERROR: PWM Timers Failed!\r\n");
        while(1);
    }
    UART_Print("PWM Timers Initialized!\r\n");

    PIDConfig pitchConfig = {2.0f, 0.0f, 0.5f, -400.0f, 400.0f, 100.0f};
    PIDConfig rollConfig  = {2.0f, 0.0f, 0.5f, -400.0f, 400.0f, 100.0f};

    PIDController pidPitch(pitchConfig);
    PIDController pidRoll(rollConfig);

    HAL::IBusReceiver receiver;
    receiver.init();
    UART_Print("Receiver Initialized!\r\n");
    UART_Print("Starting Flight Loop...\r\n");

    uint32_t lastLoopTime = HAL_GetTick();
    uint32_t lastPrintTime = HAL_GetTick();

    while (1) {
        uint32_t currentTime = HAL_GetTick();
        if (currentTime - lastLoopTime >= 2) {
            float dt = (currentTime - lastLoopTime) / 1000.0f;
            lastLoopTime = currentTime; 
            if (dt <= 0.001f) dt = 0.001f; 

            uint16_t fakeThrottle = 1000 + (currentTime % 2000) / 4; 
            uint16_t fakePitch = 1500 + 500 * sin(currentTime / 500.0f); 
            uint16_t fakeAux1 = ((currentTime / 1500) % 2 == 0) ? 1000 : 2000;
            simulateIBUS(receiver, fakeThrottle, fakePitch, fakeAux1);

            Core::ReceiverData rcData = receiver.getRCData();

            if (!receiver.isConnected()) {
                rcData.throttle = 1000; 
            }

            gyro.update();
            IMUData data = gyro.getData();

            float targetPitch = mapFloat(rcData.pitch, 1000.0f, 2000.0f, -30.0f, 30.0f);
            float targetRoll  = mapFloat(rcData.roll,  1000.0f, 2000.0f, -30.0f, 30.0f);

            float pitchCorrection = pidPitch.calculate(targetPitch, data.pitch, dt);
            float rollCorrection  = pidRoll.calculate(targetRoll,  data.roll, dt);

            bool isArmed = (rcData.aux1 > 1500);
            uint16_t baseThrottle = 1000;
            MotorSpeeds speeds = {1000, 1000, 1000, 1000};
            
            if (isArmed) {
                baseThrottle = rcData.throttle;
                speeds = MotorMixer::mix(baseThrottle, pitchCorrection, rollCorrection);
            } else {
                pidPitch.reset();
                pidRoll.reset();
            }

            pwm.setMotorSpeeds(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight);

            if (currentTime - lastPrintTime >= 100) {
                char msg[120];
                sprintf(msg, "ARM: %d | Thr: %d | Pit: %d || M1: %d | M3: %d\r\n", 
                        isArmed, rcData.throttle, rcData.pitch, speeds.frontLeft, speeds.rearLeft);
                UART_Print(msg);
                lastPrintTime = currentTime; 
            }
        }
    }
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}