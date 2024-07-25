#include "dryve_d1_bridge/d1.hpp"

#include <functional>

namespace dryve_d1_bridge
{
  // Initialize the static class variables
  const std::string_view D1::ERROR_MESSAGE_E01_CONFIGURATION = "E01 Error Configuration";
  const std::string_view D1::ERROR_MESSAGE_E02_MOTOR_OVER_CURRENT = "E02 Motor Over-Current";
  const std::string_view D1::ERROR_MESSAGE_E03_ENCODER_OVER_CURRENT = "E03 Encoder Over-Current";
  const std::string_view D1::ERROR_MESSAGE_E04_OUTPUT_OVER_CURRENT = "E04 10 V Output Over Current";
  const std::string_view D1::ERROR_MESSAGE_E05_IO_SUPPLY_LOW = "E05 I/O Supply Low";
  const std::string_view D1::ERROR_MESSAGE_E06_LOGIC_SUPPLY_LOW = "E06 Logic Supply Low";
  const std::string_view D1::ERROR_MESSAGE_E07_LOGIC_SUPPLY_HIGH = "E07 Logic Supply High";
  const std::string_view D1::ERROR_MESSAGE_E08_LOAD_SUPPLY_LOW = "E08 Load Supply Low";
  const std::string_view D1::ERROR_MESSAGE_E09_LOAD_SUPPLY_HIGH = "E09 Load Supply High";
  const std::string_view D1::ERROR_MESSAGE_E10_TEMPERATURE_HIGH = "E10 Temperature High";
  const std::string_view D1::ERROR_MESSAGE_E11_FOLLOWING_ERROR = "E11 Following Error";
  const std::string_view D1::ERROR_MESSAGE_E12_LIMIT_SWITCH = "E12 Limit Switch";
  const std::string_view D1::ERROR_MESSAGE_E13_HALL_SENSOR = "E13 Hall Sensor";
  const std::string_view D1::ERROR_MESSAGE_E14_ENCODER = "E14 Encoder";
  const std::string_view D1::ERROR_MESSAGE_E15_ENCODER_CHANNEL_A = "E15 Encoder Channel A";
  const std::string_view D1::ERROR_MESSAGE_E16_ENCODER_CHANNEL_B = "E16 Encoder Channel B";
  const std::string_view D1::ERROR_MESSAGE_E17_ENCODER_CHANNEL_I = "E17 Encoder Channel I";
  const std::string_view D1::ERROR_MESSAGE_E21_BREAKING_RESISTOR_OVERLOAD = "E21 Braking Resistor Overload";

  D1::D1(std::string ip_address, int port, std::unique_ptr<ISocketWrapper> socket_wrapper)
  {
    _socket_wrapper = std::move(socket_wrapper);

    start_connection(ip_address, port);
  }

  void D1::set_debug_mode_on()
  {
    _debug = true;
  }

  void D1::set_debug_mode_off()
  {
    _debug = false;
  }

  void D1::start_connection(std::string ip_address, int port)
  {
    // create socket
    if ((this->sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
      perror("socket failed");
      exit(EXIT_FAILURE);
    }
    else
    {
      std::cout << "Socket created: " << this->sock << "\n";
    }

    // Fill in a hint structure
    sockaddr_in hint{};
    hint.sin_family = AF_INET;
    hint.sin_port = htons(port);

    if (inet_pton(AF_INET, ip_address.c_str(), &hint.sin_addr) <= 0)
    {
      perror("inet_pton failed");
      exit(EXIT_FAILURE);
    }
    else
    {
      std::cout << "inet_pton succeeded!\n";
    }

    // connect to server
    if (connect(sock, (sockaddr *) &hint, sizeof(hint)) < 0)
    {
      perror("connect failed");
      exit(EXIT_FAILURE);
    }
    else
    {
      std::cout << "Connected to the D1!\n";
    }
  }

  void D1::close_connection()
  {
    set_profile_velocity(0, D1_DEFAULT_VELOCITY, D1_DEFAULT_ACCELERATION);
    wait_for_dryve_ready_state();
    set_dryve_shutdown_state();

    close(this->sock);
  }

  std::string_view D1::send_command_telegram(std::vector<unsigned char> telegram, int value)
  {
    unsigned char array_of_byte[4];
    unsigned char recv_buffer[19];
    unsigned char handshake[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // Swap the object and subindex bytes of the handshake to the bytes of the send telegram
    handshake[12] = telegram[12];
    handshake[13] = telegram[13];
    handshake[14] = telegram[14];
    // Conversion of the entered integer value to 4 bytes
    memcpy(array_of_byte, &value, sizeof(value));
    telegram[19] = array_of_byte[0];
    telegram[20] = array_of_byte[1];
    telegram[21] = array_of_byte[2];
    telegram[22] = array_of_byte[3];

    unsigned int send_result =
        _socket_wrapper->sending(sock, (char *) telegram.data(), telegram.size() / sizeof(telegram[0]), 0);

    if (send_result == telegram.size())
    {
      // Wait for response
      std::memset(recv_buffer, 0, 19);
      int bytes_received = _socket_wrapper->receiving(sock, (char *) recv_buffer, 19, 0);
      if (bytes_received > 0)
      {
        if (_debug == true)
        {
          std::cout << "Bytes received: " << bytes_received << std::endl;
          for (int i = 0; i < bytes_received; i++)
          {
            // Echo response to console
            printf("%d ", recv_buffer[i]);
          }
          std::cout << std::endl;
        }

        std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
        while (std::equal(std::begin(recv_buffer), std::end(recv_buffer), std::begin(handshake)) != true)
        {
          if (check_if_handshake_timeout_occurring(start_time))
          {
            return ERROR_MSG_HANDSHAKE_TIMEOUT;
          }
        }
        if (_debug == true)
        {
          std::cout << "Telegram send correctly!\n";
        }
        return std::string_view();
      }
      else
      {
        return ERROR_MSG_ZERO_BYTES_RECEIVED;
      }
    }
    else
    {
      return ERROR_MSG_WRONG_SEND_RESULT;
    }
  }

  bool D1::check_if_handshake_timeout_occurring(std::chrono::time_point<std::chrono::steady_clock> start_time)
  {
    auto elapsed_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time);

    if (_debug)
    {
      std::cout << "Wait for handshake! ";
      std::cout << "elapsed_time.count(): " << elapsed_time.count() << "\n";
    }
    if (elapsed_time.count() >= WAIT_FOR_HANDSHAKE_DURATION_MS)
    {
      std::cout << "Error: A Handshake timeout occurred! Waited for " << elapsed_time.count() << "miliseconds\n";
      return true;
    }
    return false;
  }

  std::string_view D1::wait_for_response_to_equal_handshake(std::vector<char> handshake)
  {
    unsigned char recv_buffer[19];
    // Wait for response
    std::memset(recv_buffer, 0, 19);
    int bytes_received = _socket_wrapper->receiving(sock, (char *) recv_buffer, 19, 0);
    if (bytes_received > 0)
    {
      if (_debug == true)
      {
        std::cout << "Bytes received: " << bytes_received << std::endl;
        for (int i = 0; i < bytes_received; i++)
        {
          // Print response to console for debugging
          printf("%d ", recv_buffer[i]);
        }
        std::cout << std::endl;
      }

      std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
      while (std::equal(std::begin(recv_buffer), std::end(recv_buffer), std::begin(handshake)) != true)
      {
        if (check_if_handshake_timeout_occurring(start_time))
        {
          return ERROR_MSG_HANDSHAKE_TIMEOUT;
        }
      }

      if (_debug == true)
      {
        std::cout << "Telegram send correctly!\n";
      }
      return std::string_view();
    }
    else
    {
      return ERROR_MSG_WAIT_FOR_RESPONSE_TO_EQUAL_HANDSHAKE;
    }
  }

  std::string_view D1::send_constant_set_command(const std::vector<unsigned char> telegram)
  {
    std::vector<char> handshake = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // Swap the object and subindex bytes of the handshake to the bytes of the send telegram
    handshake[12] = telegram[12];
    handshake[13] = telegram[13];
    handshake[14] = telegram[14];

    unsigned int send_result =
        _socket_wrapper->sending(sock, (char *) telegram.data(), telegram.size() / sizeof(telegram[0]), 0);

    if (send_result == telegram.size())
    {
      return wait_for_response_to_equal_handshake(handshake);
    }
    else
    {
      return ERROR_MSG_WRONG_SEND_RESULT;
    }
  }

  void D1::read_command_to_recv_buffer(const unsigned char telegram[], unsigned int array_size)
  {
    unsigned int send_result = _socket_wrapper->sending(sock, (char *) telegram, array_size / sizeof(telegram[0]), 0);
    if (send_result == array_size)
    {
      write_response_to_recv_buffer();
    }
    else
    {
      perror("Can't send telegram to D1, Err: ");
      exit(EXIT_FAILURE);
    }
  }

  int D1::write_response_to_recv_buffer()
  {
    // Wait for response
    std::memset(_recv_buffer, 0, sizeof(_recv_buffer));
    int bytes_received = _socket_wrapper->receiving(sock, (char *) _recv_buffer, sizeof(_recv_buffer), 0);
    if (bytes_received > 0)
    {
      if (_debug == true)
      {
        std::cout << "Bytes received: " << bytes_received << std::endl;
        for (int i = 0; i < bytes_received; i++)
        {
          // Echo response to console
          printf("%d ", _recv_buffer[i]);
        }
        std::cout << std::endl;
      }
      return bytes_received;
    }
    else
    {
      return bytes_received;
    }
  }

  int D1::get_response_from_socket()
  {
    int bytes_received = write_response_to_recv_buffer();
    if (bytes_received > 0)
    {
      return four_bytes_to_int(_recv_buffer);
    }
    else
    {
      return 0;
    }
  }

  int D1::read_object_value(char object_index_1, char object_index_2, int subindex)
  {
    _read_buffer[12] = int(object_index_1);
    _read_buffer[13] = int(object_index_2);
    _read_buffer[14] = subindex;

    unsigned int send_result =
        _socket_wrapper->sending(sock, (char *) _read_buffer, sizeof(_read_buffer) / sizeof(_read_buffer[0]), 0);

    if (send_result == sizeof(_read_buffer))
    {
      return get_response_from_socket();
    }
    else
    {
      perror("Can't send telegram to D1, Err: ");
      exit(EXIT_FAILURE);
    }
  }

  float D1::get_si_unit_factor()
  {
    read_command_to_recv_buffer(_READ_SI_UNIT_FACTOR, sizeof(_READ_SI_UNIT_FACTOR));
    // for further informations please see manual chapter "Detailed description Motion Control Object" Object 60A8h
    // and Object 6092h

    if (_recv_buffer[21] == MOVEMENT_TYPE_LINEAR)
    {
      return get_si_unit_factor_for_linear_movement();
    }
    else if (_recv_buffer[21] == MOVEMENT_TYPE_ROTARY)
    {
      return get_si_unit_factor_for_rotary_movement();
    }
    else
    {
      return 0;
    }
  }

  float D1::get_si_unit_factor_for_linear_movement()
  {
    // Equation to calculate the multiplication factor from the received byte 3 of object 60A8h
    if (_recv_buffer[22] < 5)
    {
      float si_unit_factor = (std::pow(10, -3) / std::pow(10, _recv_buffer[22]));
      return si_unit_factor;
    }
    else
    {
      float si_unit_factor = (std::pow(10, -3) / std::pow(10, _recv_buffer[22] - 256));
      return si_unit_factor;
    }
  }

  float D1::get_si_unit_factor_for_rotary_movement()
  {
    // Equation to calculate the multiplication factor from the received byte 3 of object 60A8h
    if (_recv_buffer[22] < 5)
    {
      float si_unit_factor = (1 / std::pow(10, _recv_buffer[22]));
      return si_unit_factor;
    }
    else
    {
      float si_unit_factor = (1 / std::pow(10, _recv_buffer[22] - 256));
      return si_unit_factor;
    }
  }

  int D1::four_bytes_to_int(unsigned char data[])
  {
    // Conversion from 4 received bytes to integer
    int int_value;
    unsigned char buffer[4]{};
    buffer[0] = data[19];
    buffer[1] = data[20];
    buffer[2] = data[21];
    buffer[3] = data[22];

    memcpy(&int_value, buffer, sizeof(int));
    return int_value;
  }

  std::string_view D1::check_for_dryve_error()
  {
    read_command_to_recv_buffer(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
    if (std::equal(std::begin(_STATUS_ERROR), std::end(_STATUS_ERROR), std::begin(_recv_buffer)) ||
        std::equal(std::begin(_STATUS_ERROR_2), std::end(_STATUS_ERROR_2), std::begin(_recv_buffer)) ||
        std::equal(std::begin(_STATUS_ERROR_3), std::end(_STATUS_ERROR_3), std::begin(_recv_buffer)) ||
        std::equal(std::begin(_STATUS_ERROR_4), std::end(_STATUS_ERROR_4), std::begin(_recv_buffer)))
    {
      // Read the Error Code and print it in the console
      int error_code = read_object_value(0x60, 0x3f);

      switch (error_code)
      {
        case ERROR_E01_CONFIGURATION:
          return ERROR_MESSAGE_E01_CONFIGURATION;
          break;

        case ERROR_E02_MOTOR_OVER_CURRENT:
          return ERROR_MESSAGE_E02_MOTOR_OVER_CURRENT;
          break;

        case ERROR_E03_ENCODER_OVER_CURRENT:
          return ERROR_MESSAGE_E03_ENCODER_OVER_CURRENT;
          break;

        case ERROR_E04_OUTPUT_OVER_CURRENT:
          return ERROR_MESSAGE_E04_OUTPUT_OVER_CURRENT;
          break;

        case ERROR_E05_IO_SUPPLY_LOW:
          return ERROR_MESSAGE_E05_IO_SUPPLY_LOW;
          break;

        case ERROR_E06_LOGIC_SUPPLY_LOW:
          return ERROR_MESSAGE_E06_LOGIC_SUPPLY_LOW;
          break;

        case ERROR_E07_LOGIC_SUPPLY_HIGH:
          return ERROR_MESSAGE_E07_LOGIC_SUPPLY_HIGH;
          break;

        case ERROR_E08_LOAD_SUPPLY_LOW:
          return ERROR_MESSAGE_E08_LOAD_SUPPLY_LOW;
          break;

        case ERROR_E09_LOAD_SUPPLY_HIGH:
          return ERROR_MESSAGE_E09_LOAD_SUPPLY_HIGH;
          break;

        case ERROR_E10_TEMPERATURE_HIGH:
          return ERROR_MESSAGE_E10_TEMPERATURE_HIGH;
          break;

        case ERROR_E11_FOLLOWING_ERROR:
          return ERROR_MESSAGE_E11_FOLLOWING_ERROR;
          break;

        case ERROR_E12_LIMIT_SWITCH:
          return ERROR_MESSAGE_E12_LIMIT_SWITCH;
          break;

        case ERROR_E13_HALL_SENSOR:
          return ERROR_MESSAGE_E13_HALL_SENSOR;
          break;

        case ERROR_E14_ENCODER:
          return ERROR_MESSAGE_E14_ENCODER;
          break;

        case ERROR_E15_ENCODER_CHANNEL_A:
          return ERROR_MESSAGE_E15_ENCODER_CHANNEL_A;
          break;

        case ERROR_E16_ENCODER_CHANNEL_B:
          return ERROR_MESSAGE_E16_ENCODER_CHANNEL_B;
          break;

        case ERROR_E17_ENCODER_CHANNEL_I:
          return ERROR_MESSAGE_E17_ENCODER_CHANNEL_I;
          break;

        case ERROR_E21_BREAKING_RESISTOR_OVERLOAD:
          return ERROR_MESSAGE_E21_BREAKING_RESISTOR_OVERLOAD;
          break;

        default:
          break;
      }
    }

    return std::string_view();
  }

  std::string_view D1::wait_for_dryve_ready_state()
  {
    do
    {
      read_command_to_recv_buffer(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      std::string_view error_message = check_for_dryve_error();
      if (_debug == true)
      {
        std::cout << "Waiting for the Movement to be finished!\n";
      }
      if (!error_message.empty())
      {
        return error_message;
      }

    } while (std::equal(std::begin(_STATUS_READY), std::end(_STATUS_READY), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_2), std::end(_STATUS_READY_2), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_5), std::end(_STATUS_READY_5), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_6), std::end(_STATUS_READY_6), std::begin(_recv_buffer)) != true);
    return std::string_view();
  }

  std::string_view D1::wait_for_homing()
  {
    do
    {
      read_command_to_recv_buffer(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      std::string_view error_message = check_for_dryve_error();
      if (_debug == true)
      {
        std::cout << "Waiting for the Homing to be finished!\n";
      }
      if (!error_message.empty())
      {
        return error_message;
      }

    } while (std::equal(std::begin(_STATUS_READY), std::end(_STATUS_READY), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_2), std::end(_STATUS_READY_2), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_3), std::end(_STATUS_READY_3), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_4), std::end(_STATUS_READY_4), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_5), std::end(_STATUS_READY_5), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_6), std::end(_STATUS_READY_6), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_7), std::end(_STATUS_READY_7), std::begin(_recv_buffer)) != true);
    return std::string_view();
  }

  std::string_view D1::set_dryve_shutdown_state()
  {
    std::string_view error_msg = send_constant_set_command(_SEND_SHUTDOWN);
    if (!error_msg.empty())
    {
      return error_msg;
    }

    do
    {
      read_command_to_recv_buffer(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      if (_debug == true)
      {
        std::cout << "Waiting for the Shutdown!\n";
      }

    } while (
        std::equal(std::begin(_STATUS_SHUTDOWN), std::end(_STATUS_SHUTDOWN), std::begin(_recv_buffer)) != true &&
        std::equal(std::begin(_STATUS_SHUTDOWN_2), std::end(_STATUS_SHUTDOWN_2), std::begin(_recv_buffer)) != true &&
        std::equal(std::begin(_STATUS_SHUTDOWN_3), std::end(_STATUS_SHUTDOWN_3), std::begin(_recv_buffer)) != true);
    return std::string_view();
  }

  std::string_view D1::set_dryve_switch_on_state()
  {
    std::string_view error_msg = send_constant_set_command(_SEND_SWITCH_ON);
    if (!error_msg.empty())
    {
      return error_msg;
    }

    do
    {
      read_command_to_recv_buffer(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      if (_debug == true)
      {
        std::cout << "Waiting for Switch On!\n";
      }

    } while (
        std::equal(std::begin(_STATUS_SWITCH_ON), std::end(_STATUS_SWITCH_ON), std::begin(_recv_buffer)) != true &&
        std::equal(std::begin(_STATUS_SWITCH_ON_2), std::end(_STATUS_SWITCH_ON_2), std::begin(_recv_buffer)) != true &&
        std::equal(std::begin(_STATUS_SWITCH_ON_3), std::end(_STATUS_SWITCH_ON_3), std::begin(_recv_buffer)) != true);
    return std::string_view();
  }

  std::string_view D1::set_dryve_operation_enable_state()
  {
    std::string_view error_msg = send_constant_set_command(_SEND_OPERATION_ENABLE);
    if (!error_msg.empty())
    {
      return error_msg;
    }

    do
    {
      read_command_to_recv_buffer(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      if (_debug == true)
      {
        std::cout << "Waiting for enable Operation!\n";
      }

    } while (std::equal(std::begin(_STATUS_OPERATION_ENABLE),
                        std::end(_STATUS_OPERATION_ENABLE),
                        std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_OPERATION_ENABLE_2),
                        std::end(_STATUS_OPERATION_ENABLE_2),
                        std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_OPERATION_ENABLE_3),
                        std::end(_STATUS_OPERATION_ENABLE_3),
                        std::begin(_recv_buffer)) != true);
    return std::string_view();
  }

  std::string_view D1::reset_dryve_status()
  {
    return send_constant_set_command(_RESET_DRYVE_STATUS);
  }

  std::string_view D1::run_dryve_state_machine()
  {
    std::vector<std::function<std::string_view()>> commands = {[&]()
                                                               {
                                                                 return send_constant_set_command(_SEND_RESET_ERROR);
                                                               },
                                                               [&]()
                                                               {
                                                                 return send_constant_set_command(_SEND_RESET_ARRAY);
                                                               },
                                                               [&]()
                                                               {
                                                                 return check_for_dryve_error();
                                                               },
                                                               [&]()
                                                               {
                                                                 return reset_dryve_status();
                                                               },
                                                               [&]()
                                                               {
                                                                 return set_dryve_shutdown_state();
                                                               },
                                                               [&]()
                                                               {
                                                                 return set_dryve_switch_on_state();
                                                               },
                                                               [&]()
                                                               {
                                                                 return set_dryve_operation_enable_state();
                                                               }};

    for (const auto &cmd : commands)
    {
      std::string_view error_msg = cmd();
      if (!error_msg.empty())
      {
        return error_msg;
      }
    }

    return std::string_view();
  }

  std::string_view D1::set_si_unit_factor(si_unit_factor si_unit_factor, bool is_movement_type_linear)
  {
    unsigned char recv_buffer[19];
    unsigned char handshake[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // Swap the object and subindex bytes of the handshake to the bytes of the send telegram
    handshake[12] = _send_si_unit_factor[12];
    handshake[13] = _send_si_unit_factor[13];
    handshake[14] = _send_si_unit_factor[14];

    _send_si_unit_factor[19] = 0;
    _send_si_unit_factor[20] = 0;
    if (is_movement_type_linear)
    {
      _send_si_unit_factor[21] = MOVEMENT_TYPE_LINEAR;
    }
    else
    {
      _send_si_unit_factor[21] = MOVEMENT_TYPE_ROTARY;
    }
    _send_si_unit_factor[22] = static_cast<unsigned char>(si_unit_factor);

    unsigned int send_result =
        _socket_wrapper->sending(sock, (char *) _send_si_unit_factor, sizeof(_send_si_unit_factor), 0);

    if (send_result == sizeof(_send_si_unit_factor))
    {
      // Wait for response
      std::memset(recv_buffer, 0, 19);
      int bytes_received = _socket_wrapper->receiving(sock, (char *) recv_buffer, 19, 0);
      if (bytes_received > 0)
      {
        if (_debug == true)
        {
          std::cout << "Bytes received: " << bytes_received << std::endl;
          for (int i = 0; i < bytes_received; i++)
          {
            // Echo response to console
            printf("%d ", recv_buffer[i]);
          }
          std::cout << std::endl;
        }

        std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
        while (std::equal(std::begin(recv_buffer), std::end(recv_buffer), std::begin(handshake)) != true)
        {
          if (check_if_handshake_timeout_occurring(start_time))
          {
            return ERROR_MSG_HANDSHAKE_TIMEOUT;
          }
        }
        if (_debug == true)
        {
          std::cout << "Telegram send correctly!\n";
        }
        return std::string_view();
      }
      else
      {
        return ERROR_MSG_ZERO_BYTES_RECEIVED;
      }
    }
    else
    {
      return ERROR_MSG_WRONG_SEND_RESULT;
    }
  }

  std::string_view D1::set_dryve_mode_of_operation(unsigned char mode)
  {
    unsigned char status_mode_display[] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode};
    _send_mode_of_operation[19] = mode;

    std::string_view error_msg = send_constant_set_command(_send_mode_of_operation);
    if (!error_msg.empty())
    {
      return error_msg;
    }

    do
    {
      read_command_to_recv_buffer(_READ_MODES_DISPLAY, sizeof(_READ_MODES_DISPLAY));
      if (_debug == true)
      {
        std::cout << "Waiting for the Mode of Operation to be set! \n";
      }

    } while (std::equal(std::begin(status_mode_display), std::end(status_mode_display), std::begin(_recv_buffer)) !=
             true);

    return std::string_view();
  }

  std::string_view D1::start_dryve_homing(float switch_velo, float zero_velo, float homing_acc)
  {
    float si_factor = get_si_unit_factor();

    std::vector<std::function<std::string_view()>> commands = {
        [&]()
        {
          return set_dryve_mode_of_operation(6);
        },
        [&]()
        {
          return send_command_telegram(_send_switch_velocity, switch_velo * si_factor);
        },
        [&]()
        {
          return send_command_telegram(_send_zero_velocity, zero_velo * si_factor);
        },
        [&]()
        {
          return send_command_telegram(_send_homing_acceleration, homing_acc * si_factor);
        },
        [&]()
        {
          return check_for_dryve_error();
        },
        [&]()
        {
          return send_constant_set_command(_SEND_RESET_START);
        },
        [&]()
        {
          return send_constant_set_command(_SEND_START_MOVEMENT);
        },
        [&]()
        {
          return wait_for_homing();
        }};

    for (const auto &cmd : commands)
    {
      std::string_view error_msg = cmd();
      if (!error_msg.empty())
      {
        return error_msg;
      }
    }

    return std::string_view();
  }

  std::string_view D1::move_profile_to_absolute_position(float position, float velo, float accel, float decel)
  {
    float si_factor = get_si_unit_factor();

    std::vector<std::function<std::string_view()>> commands = {
        [&]()
        {
          return set_dryve_mode_of_operation(1);
        },
        [&]()
        {
          return send_command_telegram(_send_target_position, position * si_factor);
        },
        [&]()
        {
          return send_command_telegram(_send_profile_velocity, velo * si_factor);
        },
        [&]()
        {
          return send_command_telegram(_send_profile_acceleration, accel * si_factor);
        },
        [&]()
        {
          return send_command_telegram(_send_profile_deceleration, decel * si_factor);
        },
        [&]()
        {
          return check_for_dryve_error();
        },
        [&]()
        {
          return send_constant_set_command(_SEND_RESET_START_ABS);
        },
        [&]()
        {
          return send_constant_set_command(_SEND_START_MOVEMENT);
        },
        [&]()
        {
          return wait_for_dryve_ready_state();
        }};

    for (const auto &cmd : commands)
    {
      std::string_view error_msg = cmd();
      if (!error_msg.empty())
      {
        return error_msg;
      }
    }

    return std::string_view();
  }

  std::string_view D1::move_profile_to_relative_position(float position, float velo, float accel, float decel)
  {
    float si_factor = get_si_unit_factor();

    std::vector<std::function<std::string_view()>> commands = {
        [&]()
        {
          return set_dryve_mode_of_operation(1);
        },
        [&]()
        {
          return send_command_telegram(_send_target_position, position * si_factor);
        },
        [&]()
        {
          return send_command_telegram(_send_profile_velocity, velo * si_factor);
        },
        [&]()
        {
          return send_command_telegram(_send_profile_acceleration, accel * si_factor);
        },
        [&]()
        {
          return send_command_telegram(_send_profile_deceleration, decel * si_factor);
        },
        [&]()
        {
          return check_for_dryve_error();
        },
        [&]()
        {
          return send_constant_set_command(_SEND_RESET_START_REL);
        },
        [&]()
        {
          return send_constant_set_command(_SEND_START_MOVEMENT_REL);
        },
        [&]()
        {
          return wait_for_dryve_ready_state();
        }};

    for (const auto &cmd : commands)
    {
      std::string_view error_msg = cmd();
      if (!error_msg.empty())
      {
        return error_msg;
      }
    }

    return std::string_view();
  }

  std::string_view D1::set_profile_velocity(float velo, float accel, float decel)
  {
    float si_factor = get_si_unit_factor();

    std::vector<std::function<std::string_view()>> commands = {
        [&]()
        {
          return set_dryve_mode_of_operation(3);
        },
        [&]()
        {
          return send_command_telegram(_send_profile_acceleration, accel * si_factor);
        },
        [&]()
        {
          return send_command_telegram(_send_profile_deceleration, decel * si_factor);
        },
        [&]()
        {
          return check_for_dryve_error();
        },
        [&]()
        {
          return send_command_telegram(_send_target_velocity, velo * si_factor);
        }};

    for (const auto &cmd : commands)
    {
      std::string_view error_msg = cmd();
      if (!error_msg.empty())
      {
        return error_msg;
      }
    }

    return std::string_view();
  }

}   // namespace dryve_d1_bridge
