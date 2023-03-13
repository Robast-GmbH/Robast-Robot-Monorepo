#ifndef DRYVE_D1_GATE__D1_HPP_
#define DRYVE_D1_GATE__D1_HPP_

#include <arpa/inet.h>
#include <sys/socket.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>

#include "dryve_d1_gate/i_socket_wrapper.hpp"

#define ERROR_E01_CONFIGURATION              25376
#define ERROR_E02_MOTOR_OVER_CURRENT         8992
#define ERROR_E03_ENCODER_OVER_CURRENT       8977
#define ERROR_E04_OUTPUT_OVER_CURRENT        8978
#define ERROR_E05_IO_SUPPLY_LOW              20756
#define ERROR_E06_LOGIC_SUPPLY_LOW           12834
#define ERROR_E07_LOGIC_SUPPLY_HIGH          12562
#define ERROR_E08_LOAD_SUPPLY_LOW            12833
#define ERROR_E09_LOAD_SUPPLY_HIGH           12817
#define ERROR_E10_TEMPERATURE_HIGH           17168
#define ERROR_E11_FOLLOWING_ERROR            34321
#define ERROR_E12_LIMIT_SWITCH               65280
#define ERROR_E13_HALL_SENSOR                29446
#define ERROR_E14_ENCODER                    29445
#define ERROR_E15_ENCODER_CHANNEL_A          65281
#define ERROR_E16_ENCODER_CHANNEL_B          65282
#define ERROR_E17_ENCODER_CHANNEL_I          65283
#define ERROR_E21_BREAKING_RESISTOR_OVERLOAD 28944

#define MOVEMENT_TYPE_LINEAR 0x01
#define MOVEMENT_TYPE_ROTARY 0x41

namespace dryve_d1_gate
{
  struct CommandTelegram
  {
    std::vector<unsigned char> telegram;
    int value;
  };

  class D1
  {
   public:
    int sock = 0;

    D1(std::string ip_address, int port, std::unique_ptr<ISocketWrapper> socket_wrapper);

    void set_debug_mode_on();

    void set_debug_mode_off();

    /***********************************************/
    /* Functions to receive information from the D1*/
    /***********************************************/

    void read_command_to_recv_buffer(const unsigned char telegram[], unsigned int array_size);

    int read_object_value(char object_index_1, char object_index_2, int subindex = 0);

    std::string check_for_dryve_error();

    std::string wait_for_dryve_ready_state();

    std::string wait_for_homing();

    float get_si_unit_factor();

    /****************************************/
    /* Functions to send commands to the D1 */
    /****************************************/

    std::string set_dryve_shutdown_state();

    std::string set_dryve_switch_on_state();

    std::string set_dryve_operation_enable_state();

    std::string send_constant_set_command(const std::vector<unsigned char> telegram);

    std::string reset_dryve_status();

    std::string run_dryve_state_machine();

    std::string set_dryve_mode_of_operation(unsigned char mode);

    std::string start_dryve_homing(float switch_velocity, float zero_velocity, float homing_acc);

    std::string move_profile_to_absolute_position(float position, float velocity, float accel, float decel = 0);

    std::string move_profile_to_relative_position(float position, float velocity, float accel, float decel = 0);

    std::string set_profile_velocity(float velocity, float accel, float decel = 0);

    static constexpr char* ERROR_MSG_HANDSHAKE_TIMEOUT = "Error: A Handshake timeout occurred!";

    static constexpr char* ERROR_MSG_WRONG_SEND_RESULT = "Error: Trying to send command, but send_result != array_size";

   private:
    void start_connection(std::string ip_address, int port);

    int write_response_to_recv_buffer();

    int get_response_from_socket();

    std::string wait_for_response_to_equal_handshake(std::vector<char> handshake);

    std::string send_command_telegram(std::vector<unsigned char> telegram, int value);

    int four_bytes_to_int(unsigned char data[]);

    float get_si_unit_factor_for_linear_movement();

    float get_si_unit_factor_for_rotary_movement();

    bool check_if_handshake_timeout_occurring(std::chrono::time_point<std::chrono::steady_clock> start_time);

    std::unique_ptr<ISocketWrapper> _socket_wrapper;

    bool _debug;   // Variable to activate and deactivate the debug mode

    static constexpr uint16_t WAIT_FOR_HANDSHAKE_DURATION_MS = 1000;

    unsigned char _recv_buffer[23];   // Buffer variable to store the received telegram from the D1

    // Define Telegrams for frequently used objects
    // State machine
    const static inline std::vector<unsigned char> _SEND_SHUTDOWN{0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                                  0, 96, 64, 0, 0, 0,  0, 2,  6,  0};
    const static inline std::vector<unsigned char> _SEND_SWITCH_ON{0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                                   0, 96, 64, 0, 0, 0,  0, 2,  7,  0};
    const static inline std::vector<unsigned char> _SEND_OPERATION_ENABLE{0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                                          0, 96, 64, 0, 0, 0,  0, 2,  15, 0};
    const static inline std::vector<unsigned char> _RESET_DRYVE_STATUS{0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                                       0, 96, 64, 0, 0, 0,  0, 2,  0,  1};

    // Telegrams for resetting the dryve status
    const static inline std::vector<unsigned char> _SEND_RESET_ERROR{0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                                     0, 96, 64, 0, 0, 0,  0, 2,  0,  1};
    const static inline std::vector<unsigned char> _SEND_RESET_ARRAY{0, 0,  0,  0, 0, 15, 0, 43, 13,  1, 0,
                                                                     0, 96, 64, 0, 0, 0,  0, 2,  143, 0};

    // Telegrams to read status and values of objects
    static constexpr unsigned char _READ_STATUS_WORD[19] = {
        0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2};
    static constexpr unsigned char _READ_MODES_DISPLAY[19] = {
        0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1};
    static constexpr unsigned char _READ_POSITION_ACTUAL_VALUE[19] = {
        0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 100, 0, 0, 0, 0, 4};
    static constexpr unsigned char _READ_VELOCITY_ACTUAL_VALUE[19] = {
        0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 108, 0, 0, 0, 0, 4};
    static constexpr unsigned char _READ_CURRENT_ACTUAL_VALUE[19] = {
        0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 120, 0, 0, 0, 0, 4};
    static constexpr unsigned char _READ_FOLLOWING_ERROR_ACTUAL_VALUE[19] = {
        0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 244, 0, 0, 0, 0, 4};
    static constexpr unsigned char _READ_ERROR_CODE[19] = {
        0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 63, 0, 0, 0, 0, 2};
    static constexpr unsigned char _READ_SI_UNIT_FACTOR[19] = {
        0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 168, 0, 0, 0, 0, 4};
    static constexpr unsigned char _READ_CONTROLLER_TEMP[19] = {
        0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 32, 19, 0, 0, 0, 0, 4};
    unsigned char _read_buffer[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};

    // Telegrams for initial parameters
    unsigned char _send_feed_rate[23] = {0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 146, 1, 0, 0, 0, 4, 0, 0, 0, 0};
    unsigned char _send_shaft_revolutions[23] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 146, 2, 0, 0, 0, 1, 1};
    unsigned char _send_si_unit_factor[23] = {0,  0,   0, 0, 0, 13, 0, 43, 13, 0, 0, 0,
                                              96, 168, 0, 0, 0, 0,  4, 0,  0,  0, 0};

    // Telegrams to set the mode of operation
    std::vector<unsigned char> _send_mode_of_operation{0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, 0};

    // Telegrams for homing/referencing
    static constexpr unsigned char _SEND_MODE_HOMING[20] = {0, 0, 0,  0,  0, 14, 0, 43, 13, 1,
                                                            0, 0, 96, 96, 0, 0,  0, 0,  1,  6};
    std::vector<unsigned char> _send_switch_velocity{0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                     96, 153, 1, 0, 0, 0,  4, 0,  0,  0, 0};
    std::vector<unsigned char> _send_zero_velocity{0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                   96, 153, 2, 0, 0, 0,  4, 0,  0,  0, 0};
    std::vector<unsigned char> _send_homing_acceleration{0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                         96, 154, 0, 0, 0, 0,  4, 0,  0,  0, 0};

    // Telegrams for Profile Position Mode and Profile Velocity Mode
    static constexpr unsigned char _SEND_MODE_PROFILE_POSITION[20] = {0, 0, 0,  0,  0, 14, 0, 43, 13, 1,
                                                                      0, 0, 96, 96, 0, 0,  0, 0,  1,  1};
    static constexpr unsigned char _SEND_MODE_PROFILE_VELOCITY[20] = {0, 0, 0,  0,  0, 14, 0, 43, 13, 1,
                                                                      0, 0, 96, 96, 0, 0,  0, 0,  1,  3};
    std::vector<unsigned char> _send_profile_velocity = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                         96, 129, 0, 0, 0, 0,  4, 0,  0,  0, 0};
    std::vector<unsigned char> _send_profile_acceleration = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                             96, 131, 0, 0, 0, 0,  4, 0,  0,  0, 0};
    std::vector<unsigned char> _send_profile_deceleration = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                             96, 132, 0, 0, 0, 0,  4, 0,  0,  0, 0};
    std::vector<unsigned char> _send_target_velocity = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                        96, 255, 0, 0, 0, 0,  4, 0,  0,  0, 0};
    std::vector<unsigned char> _send_target_position = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                        96, 122, 0, 0, 0, 0,  4, 0,  0,  0, 0};

    // Telegrams to start a movement
    const static inline std::vector<unsigned char> _SEND_START_MOVEMENT{0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                                        0, 96, 64, 0, 0, 0,  0, 2,  31, 0};

    const static inline std::vector<unsigned char> _SEND_START_MOVEMENT_REL{0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                                            0, 96, 64, 0, 0, 0,  0, 2,  95, 0};

    const static inline std::vector<unsigned char> _SEND_RESET_START{0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                                     0, 96, 64, 0, 0, 0,  0, 2,  15, 0};
    const static inline std::vector<unsigned char> _SEND_RESET_START_ABS{0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                                         0, 96, 64, 0, 0, 0,  0, 2,  15, 0};
    const static inline std::vector<unsigned char> _SEND_RESET_START_REL{0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                                         0, 96, 64, 0, 0, 0,  0, 2,  79, 0};

    // Status Word to check if the homing or the movement was completet correctly
    static constexpr unsigned char _STATUS_READY[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                        0, 96, 65, 0, 0, 0,  0, 2,  39, 22};
    static constexpr unsigned char _STATUS_READY_2[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                          0, 96, 65, 0, 0, 0,  0, 2,  8,  6};
    static constexpr unsigned char _STATUS_READY_3[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                          0, 96, 65, 0, 0, 0,  0, 2,  8,  2};
    static constexpr unsigned char _STATUS_READY_4[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                          0, 96, 65, 0, 0, 0,  0, 2,  8,  34};
    static constexpr unsigned char _STATUS_READY_5[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                          0, 96, 65, 0, 0, 0,  0, 2,  64, 22};
    static constexpr unsigned char _STATUS_READY_6[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                          0, 96, 65, 0, 0, 0,  0, 2,  64, 18};
    static constexpr unsigned char _STATUS_READY_7[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                          0, 96, 65, 0, 0, 0,  0, 2,  64, 2};

    // Status Word to that indicates that an error occured
    static constexpr unsigned char _STATUS_ERROR[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                        0, 96, 65, 0, 0, 0,  0, 2,  8,  6};
    static constexpr unsigned char _STATUS_ERROR_2[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                          0, 96, 65, 0, 0, 0,  0, 2,  8,  38};
    static constexpr unsigned char _STATUS_ERROR_3[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                          0, 96, 65, 0, 0, 0,  0, 2,  8,  16};
    static constexpr unsigned char _STATUS_ERROR_4[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                          0, 96, 65, 0, 0, 0,  0, 2,  8,  22};

    // Status Word to check if the shutdown was done correctly
    static constexpr unsigned char _STATUS_SHUTDOWN[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                           0, 96, 65, 0, 0, 0,  0, 2,  33, 6};
    static constexpr unsigned char _STATUS_SHUTDOWN_2[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                             0, 96, 65, 0, 0, 0,  0, 2,  33, 22};
    static constexpr unsigned char _STATUS_SHUTDOWN_3[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                             0, 96, 65, 0, 0, 0,  0, 2,  33, 2};

    // Status Word to check if the switch on was done correctly
    static constexpr unsigned char _STATUS_SWITCH_ON[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                            0, 96, 65, 0, 0, 0,  0, 2,  35, 6};
    static constexpr unsigned char _STATUS_SWITCH_ON_2[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                              0, 96, 65, 0, 0, 0,  0, 2,  35, 22};
    static constexpr unsigned char _STATUS_SWITCH_ON_3[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                              0, 96, 65, 0, 0, 0,  0, 2,  35, 2};

    // Status Word to check if the operation enable was done correctly
    static constexpr unsigned char _STATUS_OPERATION_ENABLE[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                                   0, 96, 65, 0, 0, 0,  0, 2,  39, 6};
    static constexpr unsigned char _STATUS_OPERATION_ENABLE_2[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                                     0, 96, 65, 0, 0, 0,  0, 2,  39, 22};
    static constexpr unsigned char _STATUS_OPERATION_ENABLE_3[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                                     0, 96, 65, 0, 0, 0,  0, 2,  39, 2};
  };
}   // namespace dryve_d1_gate

#endif   // DRYVE_D1_GATE__D1_HPP_
