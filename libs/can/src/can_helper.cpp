#include "../include/can/can_helper.hpp"

namespace robast_can_msgs
{
  std::optional<CanMessage> decode_can_message(uint32_t msg_id,
                                               uint8_t data[],
                                               uint8_t dlc,
                                               std::vector<CanMessage> can_db_messages)
  {
    for (uint16_t can_msgs_index = 0; can_msgs_index < can_db_messages.size(); can_msgs_index++)
    {
      if (msg_id == can_db_messages[can_msgs_index].get_id())
      {
        uint64_t can_msg_data = join_together_CAN_data_bytes_from_array(data, dlc);
        std::vector<CanSignal> can_signals = assign_data_to_can_signals(can_msg_data, can_db_messages, can_msgs_index);
        return CanMessage(msg_id, dlc, can_signals);
      }
    }
    return std::nullopt;
  }

  std::vector<CanSignal> assign_data_to_can_signals(uint64_t can_msg_data,
                                                    std::vector<CanMessage> can_db_messages,
                                                    uint16_t can_msgs_index)
  {
    std::vector<CanSignal> can_signals;
    uint8_t dlc = can_db_messages[can_msgs_index].get_dlc();
    for (uint16_t i = 0; i < can_db_messages[can_msgs_index].get_can_signals().size(); i++)
    {
      uint8_t bit_start = can_db_messages[can_msgs_index].get_can_signals()[i].get_bit_start();
      uint8_t bit_length = can_db_messages[can_msgs_index].get_can_signals()[i].get_bit_length();

      uint64_t can_signal_data = (can_msg_data << bit_start) >> (64 - bit_length);

      robast_can_msgs::CanSignal can_signal = CanSignal(bit_start, bit_length, can_signal_data);
      can_signals.push_back(can_signal);
    }
    return can_signals;
  }

  CanFrame encode_can_message_into_can_frame(CanMessage can_message, std::vector<CanMessage> can_db_messages)
  {
    for (uint16_t j = 0; j < can_db_messages.size(); j++)
    {
      if (can_message.get_id() == can_db_messages[j].get_id())
      {
        uint64_t can_data = join_together_CAN_data_from_CAN_message(can_message);
        uint8_t can_data_bytes[8];
        u64_to_eight_bytes(can_data, can_data_bytes);
        return CanFrame(can_message.get_id(), can_message.get_dlc(), can_data_bytes);
      }
    }
    // Please mind: We used to return std::option<CanFrame> for this function, but this didn't work
    // due to the fact that an Object wrapped into a CanFrame will be created once temporary and then
    // moved into the storage within the std::optional<> wrapping. This causes the destructor of the
    // CanFrame class to be called twice, where there is a free() operation, which can't be called twice.
    // Therefore return just the object CanFrame and in case there is a bad argument input (wrong CAN ID)
    // we throw an exception.
    throw std::invalid_argument(
        "The CAN message that should be encoded into a CAN frame has an ID which is not yet defined in the CAN "
        "database!");
  }

  void u64_to_eight_bytes(uint64_t input, uint8_t *result)
  {
    uint64_t big_endian;
    int check_for_little_endian = 1;
    // little endian if true
    if (*(char *)&check_for_little_endian == 1)
    {
      SwapEndian<uint64_t>(input);
      big_endian = input;
    }
    else
    {
      big_endian = input;
    }

    std::memcpy(result, &big_endian, sizeof(big_endian));
  }

  uint64_t join_together_CAN_data_bytes_from_array(uint8_t data[], uint8_t dlc)
  {
    uint64_t can_data = 0;
    for (int i = 0; i < dlc; i++)
    {
      uint64_t can_byte = data[i];
      can_data = (can_byte << (7 - i) * 8) | can_data;
    }
    return can_data;
  }

  uint64_t join_together_CAN_data_from_CAN_message(CanMessage can_message)
  {
    uint64_t can_data = 0;
    for (uint8_t i = 0; i < can_message.get_can_signals().size(); i++)
    {
      uint64_t can_signal_data = can_message.get_can_signals()[i].get_data();
      can_data = can_data | (can_signal_data << (64 - can_message.get_can_signals()[i].get_bit_start() -
                                                 can_message.get_can_signals()[i].get_bit_length()));
    }
    return can_data;
  }
} // namespace robast_can_msgs
