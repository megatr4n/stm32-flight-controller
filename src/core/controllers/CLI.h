#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "PIDController.h"
#include "../../hal/uart_driver.h" 

namespace Core {
    class CommandParser {
    private:
        PIDController* pitch;
        PIDController* roll;
        PIDController* yaw;
        
        char buffer[64];
        uint8_t bufIndex = 0;

        void processCommand() {
            char axis[10];
            char paramStr[10];  
            char valueStr[20];

            if (sscanf(buffer, "SET %9s %9s %19s", axis, paramStr, valueStr) == 3) {
                float value = atof(valueStr);
                char param = paramStr[0];

                PIDController* target = nullptr;
                
                if (strcmp(axis, "PITCH") == 0) target = pitch;
                else if (strcmp(axis, "ROLL") == 0) target = roll;
                else if (strcmp(axis, "YAW") == 0) target = yaw;

                if (target) {
                    PIDConfig cfg = target->getConfig();
                    
                    if (param == 'P') cfg.kp = value;
                    else if (param == 'I') cfg.ki = value;
                    else if (param == 'D') cfg.kd = value;
                    else {
                        HAL::UART_Print("ERROR: Use P, I, or D\r\n");
                        return;
                    }
                    
                    target->setConfig(cfg);
                    
                    int val_int = (int)value;
                    int val_frac = (int)(value * 100.0f) % 100;
                    if (val_frac < 0) val_frac = -val_frac;

                    char msg[64];
                    sprintf(msg, "SUCCESS: %s %c set to %d.%02d\r\n", axis, param, val_int, val_frac);
                    HAL::UART_Print(msg);
                } else {
                    HAL::UART_Print("ERROR: Unknown axis (Use PITCH, ROLL, YAW)\r\n");
                }
            } else {
                HAL::UART_Print("ERROR: Invalid format. Use: SET PITCH P 2.5\r\n");
            }
        }

    public:
        CommandParser(PIDController* p, PIDController* r, PIDController* y) 
            : pitch(p), roll(r), yaw(y) {
            memset(buffer, 0, sizeof(buffer));
        }

        void feedChar(char c) {
            if (c == '\r' || c == '\n') {
                if (bufIndex > 0) {
                    buffer[bufIndex] = '\0';
                    processCommand();
                    bufIndex = 0;
                }
            } else if (bufIndex < 63) {
                buffer[bufIndex++] = c;
            }
        }
    };
}