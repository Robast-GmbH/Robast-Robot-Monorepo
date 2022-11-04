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
            CanMessage(uint32_t id_in, uint8_t dlc_in, std::vector<CanSignal> can_signals_in) : id_{id_in}, dlc_{dlc_in}, can_signals_{can_signals_in} {}

            /**
             * @brief A deconstructor for robast_can_msgs::CanMessage class
             */
            ~CanMessage() = default;

            /**
             * @brief A setter function for the CAN signals of the CanMessage
             */
            void set_can_signals(const std::vector<CanSignal> &can_signals);

            /**
             * @brief A getter function for the CAN signals of the CanMessage
             */
			std::vector<CanSignal> get_can_signals() const;

            /**
             * @brief A getter function for the id of the CanFrame
             */
			uint32_t get_id() const;

            /**
             * @brief A getter function for the dlc of the CanFrame
             */
			uint8_t get_dlc() const;

        protected:
            std::vector<CanSignal> can_signals_;
            uint8_t dlc_;
            uint32_t id_;
    };
}

#endif /* CAN_MESSAGE_HPP_ */