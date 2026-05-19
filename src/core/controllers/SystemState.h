#pragma once
#include <stdint.h>

namespace Core {

    enum class FlightState {
        INIT,
        DISARMED,
        ARMING_CHECK,
        ARMED,
        FAILSAFE
    };

    class SystemStateMachine {
        private:
            FlightState currentState = FlightState::INIT;
        
        public:
            void update(bool isRcConnected, bool isArmSwitchOn, uint16_t throttle) {
                if (!isRcConnected && currentState != FlightState::INIT) {
                    currentState = FlightState::FAILSAFE;
                    return;
                }
                switch (currentState) {
                    case FlightState::INIT:
                        break;

                    case FlightState::DISARMED:
                    if (isArmSwitchOn) {
                        currentState = FlightState::ARMING_CHECK;
                    }
                        break;

                    case FlightState::ARMING_CHECK:
                    if (!isArmSwitchOn) {
                        currentState = FlightState::DISARMED;
                    } else if (throttle < 1050) {
                        currentState = FlightState::ARMED;
                } else {
                    currentState = FlightState::DISARMED;
                }
                break;

                    case FlightState::ARMED:
                    if (!isArmSwitchOn) {
                        currentState = FlightState::DISARMED;
                    }
                    break;

                    case FlightState::FAILSAFE:
                    if (isRcConnected && !isArmSwitchOn) {
                        currentState = FlightState::FAILSAFE;
                    }
                    break;
            }
    }
            void notifyInitComplete() {
                if (currentState == FlightState::INIT) {
                    currentState = FlightState::DISARMED;
                }
            }

            FlightState getState() const {
                return currentState;
            }

            bool areMotorsAllowed() const {
                return currentState == FlightState::ARMED;
            }
        };
}
