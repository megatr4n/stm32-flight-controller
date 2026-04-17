#pragma once
#include <stdint.h>

namespace Core {
    struct ReceiverData {
        uint16_t throttle;
        uint16_t pitch;
        uint16_t roll;
        uint16_t yaw;

        uint16_t aux1;
        uint16_t aux2;
    };

    class I_Receiver {
    public:
        virtual ~I_Receiver() = default;
        virtual bool init() = 0;
        virtual ReceiverData getRCData() = 0;
        virtual bool isConnected() = 0; 
    };
}