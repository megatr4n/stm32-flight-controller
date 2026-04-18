#include "IBusReceiver.h"
#include "stm32f1xx_hal.h"

namespace HAL {
    IBusReceiver::IBusReceiver() {
        bufferIndex = 0;
        connected = false;
        lastPacketTime = 0;

        currentData.throttle = 1000;
        currentData.pitch = 1500;
        currentData.roll = 1500;
        currentData.yaw = 1500;
        currentData.aux1 = 1000;
        currentData.aux2 = 1000;
    }

    bool IBusReceiver::init() {
        return true;
    }

    void IBusReceiver::feedByte(uint8_t byte) {
        if (bufferIndex == 0 && byte != 0x20) return;
        
        if (bufferIndex == 1 && byte != 0x40) {
            bufferIndex = 0;
            return;
        }
        buffer[bufferIndex++] = byte;
        if (bufferIndex == 32) {
            proccesPacket();
            bufferIndex = 0;
        }
    }

    void IBusReceiver::proccesPacket() {
        uint16_t calculatedChecksum = 0xFFFF;
        for (int i = 0; i < 30; i++) {
            calculatedChecksum -= buffer[i];
        }
        uint16_t packetChecksum = buffer[30] | (buffer[31] << 8);
        
        if (calculatedChecksum != packetChecksum) return;

        currentData.roll = buffer[2] | (buffer[3] << 8);
        currentData.pitch = buffer[4] | (buffer[5] << 8);
        currentData.throttle = buffer[6] | (buffer[7] << 8);
        currentData.yaw = buffer[8] | (buffer[9] << 8);
        currentData.aux1 = buffer[10] | (buffer[11] << 8);
        currentData.aux2 = buffer[12] | (buffer[13] << 8);

        connected = true;
        lastPacketTime = HAL_GetTick();
    }

    Core::ReceiverData IBusReceiver::getRCData() {
        return currentData;
    }

    bool IBusReceiver::isConnected() {
        if (HAL_GetTick() - lastPacketTime > 500) {
            connected = false;
            currentData.throttle = 1000;
            currentData.pitch = 1500;
            currentData.roll = 1500;
            currentData.yaw = 1500;
        }
        return connected; 
    }   
}