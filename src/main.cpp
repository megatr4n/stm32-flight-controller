#define WOKWI_SIMULATION 1

#include "stm32f1xx_hal.h"
#include "hal/STM32_I2C.h"
#include "hal/uart_driver.h"
#include "devices/mpu6050.h"

#include "core/controllers/PIDController.h"
#include "core/controllers/MotorMixer.h"

#include "hal/pwm_driver.h"
#include "hal/IBusReceiver.h"

#include "core/filters/LowPassFilter.h"
#include "hal/timing.h"

#include "core/controllers/SystemState.h"
#include "core/controllers/CLI.h"

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

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t rx_buffer[64];
uint16_t rx_tail = 0;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    UART1_Init();
    
    UART_Print("\r\nFlight Controller Starting!\r\n");

    Core::SystemStateMachine stateMachine;

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
    PIDConfig yawConfig   = {3.0f, 0.0f, 0.0f, -400.0f, 400.0f, 100.0f};

    PIDController pidPitch(pitchConfig);
    PIDController pidRoll(rollConfig);
    PIDController pidYaw(yawConfig);

    LowPassFilter filterPitch(0.15f);
    LowPassFilter filterRoll(0.15f);
    LowPassFilter filterYaw(0.15f);

    HAL::IBusReceiver receiver;
    receiver.init();

    Core::CommandParser cli(&pidPitch, &pidRoll, &pidYaw);

    UART1_Start_DMA_RX(rx_buffer, 64);
    UART_Print("Receiver Initialized!\r\n");

    HAL::Timing::init();

    bool calibrationRequested = false;

    #ifndef WOKWI_SIMULATION
    uint16_t initial_cndtr = __HAL_DMA_GET_COUNTER(&HAL::hdma_usart1_rx);
    uint16_t initial_rx_head = (64 - initial_cndtr) % 64;
    while (rx_tail != initial_rx_head) {
        receiver.feedByte(rx_buffer[rx_tail]);
        rx_tail = (rx_tail + 1) % 64;
    }
    #endif

    stateMachine.notifyInitComplete(calibrationRequested);
    UART_Print("Starting Flight Loop...\r\n");

    uint32_t lastLoopTimeUs = HAL::Timing::getMicros();
    uint32_t lastPrintTimeMs = HAL_GetTick();

    while (1) {
        uint32_t currentTimeUs = HAL::Timing::getMicros();
        
        if (currentTimeUs - lastLoopTimeUs >= 2000) {
            
            float dt = (currentTimeUs - lastLoopTimeUs) / 1000000.0f;
            lastLoopTimeUs = currentTimeUs; 
            
            if (dt <= 0.001f || dt > 0.05f) dt = 0.002f; 

            #ifdef WOKWI_SIMULATION
                while (USART1->SR & USART_SR_RXNE) {
                    uint8_t incomingByte = USART1->DR;
                    
                    char echo[2] = { (char)incomingByte, '\0' };
                    UART_Print(echo);
                    
                    cli.feedChar((char)incomingByte);
                }
            #else
                uint16_t cndtr = __HAL_DMA_GET_COUNTER(&HAL::hdma_usart1_rx);
                uint16_t rx_head = (64 - cndtr) % 64; 
                
                while (rx_tail != rx_head) {
                    uint8_t incomingByte = rx_buffer[rx_tail];
                    
                    receiver.feedByte(incomingByte);
                    cli.feedChar((char)incomingByte); 
                    
                    rx_tail = (rx_tail + 1) % 64;
                }
            #endif

            Core::ReceiverData rcData = receiver.getRCData();
            bool is_connected = receiver.isConnected();

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

            gyro.update();
            IMUData data = gyro.getData();

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

            pwm.setMotorSpeeds(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight);

            uint32_t currentTimeMs = HAL_GetTick();

            static bool testCmdSent = false;
            if (!testCmdSent && currentTimeMs > 5000) {
                const char* testCmd = "SET PITCH P 3.14\r\n";
                for(int i = 0; testCmd[i] != '\0'; i++) {
                    cli.feedChar(testCmd[i]);
                }
                testCmdSent = true;
            }

            if (currentTimeMs - lastPrintTimeMs >= 1000) {
                char msg[140];
                sprintf(msg, "STATE: %d | Thr: %d | Pit: %d || M1: %d | M3: %d\r\n", 
                        static_cast<int>(stateMachine.getState()), rcData.throttle, (int)cleanPitch, speeds.frontLeft, speeds.rearLeft);
                UART_Print(msg);
                lastPrintTimeMs = currentTimeMs; 
            }
        }
    }
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}