#ifndef CAN_MESSAGE_HPP_
#define CAN_MESSAGE_HPP_

#include <optional>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>
#include <array>
#include <algorithm>

#include "can_signal.h"

namespace robast_can_msgs
{
    class CanMessage
    {
        public:
            /**
             * @brief A constructor for robast_can_msgs::CanMessage class
             */
            CanMessage(uint32_t id_in, uint8_t dlc_in, std::vector<CanSignal> can_signals_in) : id{id_in}, dlc{dlc_in}, can_signals_{can_signals_in} {}

            const uint32_t id;
            const uint8_t dlc;

            /**
             * @brief A setter function for the CAN signals of the CanMessage
             */
            void set_can_signals(const std::vector<CanSignal> &can_signals);

            /**
             * @brief A getter function for the CAN signals of the CanMessage
             */
			std::vector<CanSignal> get_can_signals() const;

        protected:
            std::vector<CanSignal> can_signals_;
    };
}

#endif /* CAN_MESSAGE_HPP_ */