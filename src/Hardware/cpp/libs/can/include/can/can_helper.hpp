#ifndef CAN__CAN_HELPER_HPP_
#define CAN__CAN_HELPER_HPP_

#include <algorithm>
#include <iomanip>
#include <optional>
#include <string>
#include <vector>
#include <array>
#include <bit>

#include "can/can_frame.hpp"
#include "can/can_message.hpp"

namespace robast_can_msgs
{
  /**
   * @brief Decodes CAN message from a 8 Byte bitstream
   *
   * @param msg_id The ID of the received can msg
   * @param data The array of data bytes of the received msg
   * @param dlc The number of data bytes of the received msg
   * @param can_db_messages CAN messages from CAN database to decode the message with
   * @return std::optional<CanMessage>
   */
  std::optional<CanMessage> decode_can_message(uint32_t msg_id,
                                               uint8_t data[],
                                               uint8_t dlc,
                                               std::vector<CanMessage> can_db_messages);

  /**
   * @brief Encodes CAN message into CAN_frame_t with 8 Byte bitstream
   *
   * @param can_message The can_message to be encoded
   * @param can_db_messages CAN messages from CAN database to encode the message with
   * @return std::optional<CanFrame>. Only contains a value if the can_message id exists in the can_db.
   */
  CanFrame encode_can_message_into_can_frame(CanMessage can_message, std::vector<CanMessage> can_db_messages);

  /**
   * @brief Joins together the data bytes from the CAN bus
   *
   * @param data The received CAN data
   * @param dlc The number of data bytes
   * @return 8Byte CAN data joined together
   */
  uint64_t join_together_CAN_data_bytes_from_array(uint8_t data[], uint8_t dlc);

  /**
   * @brief Joins together the CAN data that is encapsulated in the CanMessage class
   *
   * @param can_message The CAN message encapsulated into the CanMessage class
   * @return All CAN data joined into 64 bit stream
   */
  uint64_t join_together_CAN_data_from_CAN_message(CanMessage can_message);

  /**
   * @brief Converts a uint64 into 8 byte array
   *
   * @param input The uint64 to be converted
   * @param result Pointer to the 8 byte array for the result
   */
  void u64_to_eight_bytes(uint64_t input, uint8_t *result);

  /**
   * @brief Convert hex data contained in a string to an unsigned integer
   *
   * @param hex_string The input string that contains the data in hex format.
   * @return The data, that was contained in the string, as an unsigned integer.
   */
  template <typename T>
  T hex_string_to_unsigned_int(std::string hex_string)
  {
    // Mind that defining templates functions within the cpp file makes problems
    // when building with -DCMAKE_BUILD_TYPE=RelWithDebInfo because of the optimization
    // Therefore we define it within the header.
    T result;
    result = std::stoul(hex_string, nullptr, 16);
    return result;
  }

  /**
   * @brief Swap Endian
   *
   * @param val The value that's supposed to be swapped
   */
  template <typename T>
  void SwapEndian(T &val)
  {
    // Mind that defining templates functions within the cpp file makes problems
    // when building with -DCMAKE_BUILD_TYPE=RelWithDebInfo because of the optimization
    // Therefore we define it within the header.
    auto raw = std::bit_cast<std::array<uint8_t, sizeof(T)>>(val);
    std::ranges::reverse(raw);
    val = std::bit_cast<T>(raw);
  }

  /**
   * @brief Assigns the data that is contained in the uint64_t parameter to the can signals
   *
   * @param can_msg_data The uint64_t parameter that contains the data for the can signals
   * @param can_db_messages  The vector containing all can_messages that are defined in the CAN database
   * @param can_msgs_index The index of the CAN message we want to decode
   * @return A Vector of all CanSignals filled with the data that is contained in the can_msgs_data parameter
   */
  std::vector<CanSignal> assign_data_to_can_signals(uint64_t can_msg_data,
                                                    std::vector<CanMessage> can_db_messages,
                                                    uint16_t can_msgs_index);

} // namespace robast_can_msgs

#endif // CAN__CAN_HELPER_HPP_
