#ifndef CAN__CAN_SIGNAL_HPP_
#define CAN__CAN_SIGNAL_HPP_

#include <cstdint>

namespace robast_can_msgs
{
  class CanSignal
  {
   public:
    /**
     * @brief A constructor for robast_can_msgs::CanSignal class
     */
    CanSignal(uint8_t bit_start, uint8_t bit_length, uint64_t data)
        : _bit_start{bit_start}, _bit_length{bit_length}, _data{data}
    {
    }

    /**
     * @brief A destructor for robast_can_msgs::CanSignal class
     */
    ~CanSignal() = default;

    /**
     * @brief A setter function for the data of the CanSignal
     */
    void set_data(uint64_t data);

    /**
     * @brief A getter function for the data of the CanSignal
     */
    uint64_t get_data() const;

    /**
     * @brief A getter function for the bit_start of the CanSignal
     */
    uint8_t get_bit_start() const;

    /**
     * @brief A getter function for the bit_length of the CanSignal
     */
    uint8_t get_bit_length() const;

   private:
    uint8_t _bit_start;
    uint8_t _bit_length;   // number of bits for this can_signal
    uint64_t _data;
  };
}   // namespace robast_can_msgs

#endif   // CAN__CAN_SIGNAL_HPP_