#include "dryve_d1_gate/d1.hpp"

namespace dryve_d1_gate
{
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

  void D1::send_command_telegram(unsigned char telegram[], unsigned int array_size, int value)
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

    unsigned int send_result = _socket_wrapper->sending(sock, (char *) telegram, array_size / sizeof(telegram[0]), 0);

    if (send_result == array_size)
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
        while (std::equal(std::begin(recv_buffer), std::end(recv_buffer), std::begin(handshake)) != true)
        {
          std::cout << "Wait for handshake\n";
        }
        if (_debug == true)
        {
          std::cout << "Telegram send correctly!\n";
        }
      }
    }
    else
    {
      perror("Can't send telegram to D1, Err: ");
      exit(EXIT_FAILURE);
    }
  }

  void D1::wait_for_response_to_equal_handshake(std::vector<char> handshake)
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
      while (std::equal(std::begin(recv_buffer), std::end(recv_buffer), std::begin(handshake)) != true)
      {
        std::cout << "Wait for handshake\n";
        // TODO@all: Is this really smart to infinitely wait for the handshake?
      }

      if (_debug == true)
      {
        std::cout << "Telegram send correctly!\n";
      }
    }
    else
    {
      perror("Can't send telegram to D1, Err: ");
      exit(EXIT_FAILURE);
    }
  }

  void D1::send_constant_set_command(const unsigned char telegram[], unsigned int array_size)
  {
    std::vector<char> handshake = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // Swap the object and subindex bytes of the handshake to the bytes of the send telegram
    handshake[12] = telegram[12];
    handshake[13] = telegram[13];
    handshake[14] = telegram[14];

    unsigned int send_result = _socket_wrapper->sending(sock, (char *) telegram, array_size / sizeof(telegram[0]), 0);

    if (send_result == array_size)
    {
      wait_for_response_to_equal_handshake(handshake);
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
      // TODO@All: What do we do, if no byte was received?
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
      // TODO@All: What do we do, if no byte was received?
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
      // TODO@All: Do we want to exit here like this?
      perror("Can't send telegram to D1, Err: ");
      exit(EXIT_FAILURE);
    }
  }

  float D1::get_si_unit_factor()
  {
    read_command_to_recv_buffer(_READ_SI_UNIT_FACTOR, sizeof(_READ_SI_UNIT_FACTOR));
    // Read the SI Unit Position calculation of the multiplication factor when linear movement(byte 2 == 01h) is set;

    // for further informations please see manual chapter "Detailed description Motion Control Object" Object 60A8h
    // and Object 6092h
    if (_recv_buffer[21] == 1)
    {
      // Equation to calculate the multiplication factor from the recieved byte 3 of object 60A8h
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
    // Read the SI Unit Positionand calculation of the multiplication factor when rotary movement(byte 2 == 41h) is
    // set; for further informations please see manual chapter "Detailed description Motion Control Object" Object
    // 60A8h and Object 6092h
    else if (_recv_buffer[21] == 65)
    {
      // Equation to calculate the multiplication factor from the recieved byte 3 of object 60A8h
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

  std::string D1::check_for_dryve_error()
  {
    read_command_to_recv_buffer(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
    if (std::equal(std::begin(_STATUS_ERROR), std::end(_STATUS_ERROR), std::begin(_recv_buffer)) ||
        std::equal(std::begin(_STATUS_ERROR_2), std::end(_STATUS_ERROR_2), std::begin(_recv_buffer)) ||
        std::equal(std::begin(_STATUS_ERROR_3), std::end(_STATUS_ERROR_3), std::begin(_recv_buffer)) ||
        std::equal(std::begin(_STATUS_ERROR_4), std::end(_STATUS_ERROR_4), std::begin(_recv_buffer)))
    {
      // Read the Error Code and print it in the console
      int error_code = read_object_value(0x60, 0x3f);

      if (error_code == ERROR_E01_CONFIGURATION)
      {
        return "E01 Error Configuration";
      }
      if (error_code == ERROR_E02_MOTOR_OVER_CURRENT)
      {
        return "E02 Motor Over-Current";
      }
      if (error_code == ERROR_E03_ENCODER_OVER_CURRENT)
      {
        return "E03 Encoder Over-Current";
      }
      if (error_code == ERROR_E04_OUTPUT_OVER_CURRENT)
      {
        return "E04 10 V Output Over Current";
      }
      if (error_code == ERROR_E05_IO_SUPPLY_LOW)
      {
        return "E05 I/O Supply Low";
      }
      if (error_code == ERROR_E06_LOGIC_SUPPLY_LOW)
      {
        return "E06 Logic Supply Low";
      }
      if (error_code == ERROR_E07_LOGIC_SUPPLY_HIGH)
      {
        return "E07 Logic Supply High";
      }
      if (error_code == ERROR_E08_LOAD_SUPPLY_LOW)
      {
        return "E08 Load Supply Low";
      }
      if (error_code == ERROR_E09_LOAD_SUPPLY_HIGH)
      {
        return "E09 Load Supply High";
      }
      if (error_code == ERROR_E10_TEMPERATURE_HIGH)
      {
        return "E10 Temperature High";
      }
      if (error_code == ERROR_E11_FOLLOWING_ERROR)
      {
        return "E11 Following Error";
      }
      if (error_code == ERROR_E12_LIMIT_SWITCH)
      {
        return "E12 Limit Switch";
      }
      if (error_code == ERROR_E13_HALL_SENSOR)
      {
        return "E13 Hall Sensor";
      }
      if (error_code == ERROR_E14_ENCODER)
      {
        return "E14 Encoder";
      }
      if (error_code == ERROR_E15_ENCODER_CHANNEL_A)
      {
        return "E15 Encoder Channel A";
      }
      if (error_code == ERROR_E16_ENCODER_CHANNEL_B)
      {
        return "E16 Encoder Channel B";
      }
      if (error_code == ERROR_E17_ENCODER_CHANNEL_I)
      {
        return "E17 Encoder Channel I";
      }
      if (error_code == ERROR_E21_BREAKING_RESISTOR_OVERLOAD)
      {
        return "E21 Braking Resistor Overload";
      }
    }

    return std::string();
  }

  std::string D1::wait_for_dryve_ready_state()
  {
    do
    {
      read_command_to_recv_buffer(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      std::string error_message = check_for_dryve_error();
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
    return std::string();
  }

  void D1::wait_for_homing()
  {
    do
    {
      read_command_to_recv_buffer(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      check_for_dryve_error();
      if (_debug == true)
      {
        std::cout << "Waiting for the Homing to be finished!\n";
      }

    } while (std::equal(std::begin(_STATUS_READY), std::end(_STATUS_READY), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_2), std::end(_STATUS_READY_2), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_3), std::end(_STATUS_READY_3), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_4), std::end(_STATUS_READY_4), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_5), std::end(_STATUS_READY_5), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_6), std::end(_STATUS_READY_6), std::begin(_recv_buffer)) != true &&
             std::equal(std::begin(_STATUS_READY_7), std::end(_STATUS_READY_7), std::begin(_recv_buffer)) != true);
  }

  void D1::set_dryve_shutdown_state()
  {
    send_constant_set_command(_SEND_SHUTDOWN, sizeof(_SEND_SHUTDOWN));
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
  }

  void D1::set_dryve_switch_on_state()
  {
    send_constant_set_command(_SEND_SWITCH_ON, sizeof(_SEND_SWITCH_ON));
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
  }

  void D1::set_dryve_operation_enable_state()
  {
    send_constant_set_command(_SEND_OPERATION_ENABLE, sizeof(_SEND_OPERATION_ENABLE));
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
  }

  void D1::reset_dryve_status()
  {
    send_constant_set_command(_RESET_DRYVE_STATUS, sizeof(_RESET_DRYVE_STATUS));
  }

  void D1::run_dryve_state_machine()

  {
    send_constant_set_command(_send_reset_error, sizeof(_send_reset_error));
    send_constant_set_command(_send_reset_array, sizeof(_send_reset_array));
    check_for_dryve_error();
    reset_dryve_status();
    set_dryve_shutdown_state();
    set_dryve_switch_on_state();
    set_dryve_operation_enable_state();
  }

  void D1::set_dryve_mode_of_operation(unsigned char mode)
  {
    unsigned char status_mode_display[] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode};
    _send_mode_of_operation[19] = mode;
    send_constant_set_command(_send_mode_of_operation, sizeof(_send_mode_of_operation));
    do
    {
      read_command_to_recv_buffer(_READ_MODES_DISPLAY, sizeof(_READ_MODES_DISPLAY));
      if (_debug == true)
      {
        std::cout << "Waiting for the Mode of Operation to be set! \n";
      }

    } while (std::equal(std::begin(status_mode_display), std::end(status_mode_display), std::begin(_recv_buffer)) !=
             true);
  }

  void D1::start_dryve_homing(float switch_velo, float zero_velo, float homing_acc)
  {
    set_dryve_mode_of_operation(6);
    float si_factor = get_si_unit_factor();
    long switch_velocity = switch_velo * si_factor;
    long zero_velocity = zero_velo * si_factor;
    long homing_accel = homing_acc * si_factor;
    send_command_telegram(_send_switch_velocity, sizeof(_send_switch_velocity), switch_velocity);
    send_command_telegram(_send_zero_velocity, sizeof(_send_zero_velocity), zero_velocity);
    send_command_telegram(_send_homing_acceleration, sizeof(_send_homing_acceleration), homing_accel);

    // Checks if the D1 is in an error state
    check_for_dryve_error();

    // Start Movement and toggle back bit 4
    send_constant_set_command(_SEND_RESET_START, sizeof(_SEND_RESET_START));
    send_constant_set_command(_SEND_START_MOVEMENT, sizeof(_SEND_START_MOVEMENT));

    // Wait for homing to end
    wait_for_homing();
  }

  void D1::move_profile_to_absolute_position(float position, float velo, float accel, float decel)
  {
    set_dryve_mode_of_operation(1);
    float si_factor = get_si_unit_factor();
    long pos = position * si_factor;
    long velocity = velo * si_factor;
    long acc = accel * si_factor;
    long dec = decel * si_factor;
    send_command_telegram(_send_target_position, sizeof(_send_target_position), pos);
    send_command_telegram(_send_profile_velocity, sizeof(_send_profile_velocity), velocity);
    send_command_telegram(_send_profile_acceleration, sizeof(_send_profile_acceleration), acc);
    send_command_telegram(_send_profile_deceleration, sizeof(_send_profile_deceleration), dec);

    // Checks if the D1 is in an error state
    check_for_dryve_error();

    // Start Movement and toggle back bit 4
    send_constant_set_command(_SEND_RESET_START_ABS, sizeof(_SEND_RESET_START_ABS));
    send_constant_set_command(_SEND_START_MOVEMENT, sizeof(_SEND_START_MOVEMENT));

    // Wait for Movement to end
    std::cout << "Wait for Movement to end! \n";
    wait_for_dryve_ready_state();
    std::cout << "Movement ended! \n";
  }

  void D1::move_profile_to_relative_position(float position, float velo, float accel, float decel)
  {
    set_dryve_mode_of_operation(1);
    float si_factor = get_si_unit_factor();
    long pos = position * si_factor;
    long velocity = velo * si_factor;
    long acc = accel * si_factor;
    long dec = decel * si_factor;
    send_command_telegram(_send_target_position, sizeof(_send_target_position), pos);
    send_command_telegram(_send_profile_velocity, sizeof(_send_profile_velocity), velocity);
    send_command_telegram(_send_profile_acceleration, sizeof(_send_profile_acceleration), acc);
    send_command_telegram(_send_profile_deceleration, sizeof(_send_profile_deceleration), dec);

    // Checks if the D1 is in an error state
    check_for_dryve_error();

    // Start Movement and toggle back bit 4
    send_constant_set_command(_SEND_RESET_START_REL, sizeof(_SEND_RESET_START_REL));
    send_constant_set_command(_SEND_START_MOVEMENT_REL, sizeof(_SEND_START_MOVEMENT_REL));

    // Wait for Movement to end
    wait_for_dryve_ready_state();
  }

  void D1::set_profile_velocity(float velo, float accel, float decel)
  {
    set_dryve_mode_of_operation(3);
    float si_factor = get_si_unit_factor();
    long velocity = velo * si_factor;
    long acc = accel * si_factor;
    long dec = decel * si_factor;
    send_command_telegram(_send_profile_acceleration, sizeof(_send_profile_acceleration), acc);
    send_command_telegram(_send_profile_deceleration, sizeof(_send_profile_deceleration), dec);

    // Checks if the D1 is in an error state
    check_for_dryve_error();

    // Start movement by sending a target velocity value != 0 (0 for stop)
    send_command_telegram(_send_target_velocity, sizeof(_send_target_velocity), velocity);
  }

}   // namespace dryve_d1_gate
