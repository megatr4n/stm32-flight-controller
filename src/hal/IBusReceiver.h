#pragma once
#include "../core/interfaces/I_Receiver.h"
#include <stdint.h>

namespace HAL {
    class IBusReceiver : public Core::I_Receiver {
    private:
        Core::ReceiverData currentData;
        bool connected;
        uint32_t lastPacketTime;

        uint8_t buffer[32];
        uint8_t bufferIndex;

        void proccesPacket();

    public:
        IBusReceiver();
        bool init() override;
        Core::ReceiverData getRCData() override;
        bool isConnected() override;
        void feedByte(uint8_t byte);
    };
}