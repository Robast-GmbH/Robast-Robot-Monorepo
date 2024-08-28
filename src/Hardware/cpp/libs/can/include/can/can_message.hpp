#ifndef CAN__CAN_MESSAGE_HPP_
#define CAN__CAN_MESSAGE_HPP_

#include <vector>

#include "can/can_signal.hpp"

namespace robast_can_msgs
{
  class CanMessage
  {
   public:
    /**
     * @brief A constructor for robast_can_msgs::CanMessage class
     */
    CanMessage(uint32_t id, uint8_t dlc, std::vector<CanSignal> can_signals)
        : _id{id}, _dlc{dlc}, _can_signals{can_signals}
    {
    }

    /**
     * @brief A default constructor for robast_can_msgs::CanMessage class
     */
    CanMessage() = default;

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

   private:
    uint32_t _id;
    uint8_t _dlc;
    std::vector<CanSignal> _can_signals;
  };
}   // namespace robast_can_msgs

#endif   // CAN__CAN_MESSAGE_HPP_
