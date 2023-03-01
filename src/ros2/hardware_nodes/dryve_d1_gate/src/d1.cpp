#include "dryve_d1_gate/d1.hpp"

namespace dryve_d1_gate
{
  D1::D1(std::string ipAddress, int port)
  {
    start_connection(ipAddress, port);
  }

  void D1::set_debug_mode_on()
  {
    _debug = true;
  }

  void D1::set_debug_mode_off()
  {
    _debug = false;
  }

  void D1::start_connection(std::string ipAddress, int port)
  {
    // create socket
    if ((this->sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
      perror("socket failed");
      exit(EXIT_FAILURE);
    }
    else
    {
      std::cout << "Socket created!\n";
    }

    // Fill in a hint structure
    sockaddr_in hint{};
    hint.sin_family = AF_INET;
    hint.sin_port = htons(port);

    if (inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr) <= 0)
    {
      perror("inet_pton failed");
      exit(EXIT_FAILURE);
    }
    else
    {
      std::cout << "inet_pton succeeded!\n";
    }

    // connect to server
    if (connect(sock, (sockaddr*) &hint, sizeof(hint)) < 0)
    {
      perror("connect failed");
      exit(EXIT_FAILURE);
    }
    else
    {
      std::cout << "Connected to the D1!\n";
    }
  }

  void D1::send_command(unsigned char data[], unsigned int arraySize, long value)
  {
    unsigned char arrayOfByte[8];
    unsigned char recvbuffer[19];
    unsigned char handShake[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // Swap the object and subindex bytes of the handshake to the bytes of the send telegram
    handShake[12] = data[12];
    handShake[13] = data[13];
    handShake[14] = data[14];
    // Conversion of the entered integer value to 4 bytes
    memcpy(arrayOfByte, &value, sizeof(value));
    data[19] = arrayOfByte[0];
    data[20] = arrayOfByte[1];
    data[21] = arrayOfByte[2];
    data[22] = arrayOfByte[3];

    int sendResult = send(sock, (char*) data, arraySize / sizeof(data[0]), 0);

    if (sendResult == arraySize)
    {
      // Wait for response
      std::memset(recvbuffer, 0, 19);
      int bytesReceived = recv(sock, (char*) recvbuffer, 19, 0);
      if (bytesReceived > 0)
      {
        if (_debug == true)
        {
          std::cout << "Bytes received: " << bytesReceived << std::endl;
          for (int i = 0; i < bytesReceived; i++)
          {
            // Echo response to console
            printf("%d ", recvbuffer[i]);
          }
          std::cout << std::endl;
        }
        while (std::equal(std::begin(recvbuffer), std::end(recvbuffer), std::begin(handShake)) != true)
        {
          std::cout << "Wait for Handshake\n";
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

  void D1::send_set_command(const unsigned char data[], unsigned int arraySize)
  {
    // Send of const telegrams
    unsigned char recvbuffer[19];
    unsigned char handShake[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // Swap the object and subindex bytes of the handshake to the bytes of the send telegram
    handShake[12] = data[12];
    handShake[13] = data[13];
    handShake[14] = data[14];

    int sendResult = send(sock, (char*) data, arraySize / sizeof(data[0]), 0);

    if (sendResult == arraySize)
    {
      // Wait for response
      std::memset(recvbuffer, 0, 19);
      int bytesReceived = recv(sock, (char*) recvbuffer, 19, 0);
      if (bytesReceived > 0)
      {
        if (_debug == true)
        {
          std::cout << "Bytes received: " << bytesReceived << std::endl;
          for (int i = 0; i < bytesReceived; i++)
          {
            // Print response to console for debugging
            printf("%d ", recvbuffer[i]);
          }
          std::cout << std::endl;
        }
        while (std::equal(std::begin(recvbuffer), std::end(recvbuffer), std::begin(handShake)) != true)
        {
          std::cout << "Wait for Handshake\n";
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

  void D1::read_command(const unsigned char data[], unsigned int arraySize)
  {
    int sendResult = send(sock, (char*) data, arraySize / sizeof(data[0]), 0);
    if (sendResult == arraySize)
    {
      // Wait for response
      std::memset(_recvbuf, 0, sizeof(_recvbuf));
      int bytesReceived = recv(sock, (char*) _recvbuf, sizeof(_recvbuf), 0);
      if (bytesReceived > 0)
      {
        if (_debug == true)
        {
          std::cout << "Bytes received: " << bytesReceived << std::endl;
          for (int i = 0; i < bytesReceived; i++)
          {
            // Echo response to console
            printf("%d ", _recvbuf[i]);
          }
          std::cout << std::endl;
        }
      }
    }
    else
    {
      perror("Can't send telegram to D1, Err: ");
      exit(EXIT_FAILURE);
    }
  }

  int D1::read_object_value(char objectindex1, char objectindex2, int subindex)
  {
    _read_buffer[12] = int(objectindex1);
    _read_buffer[13] = int(objectindex2);
    _read_buffer[14] = subindex;

    int sendResult = send(sock, (char*) _read_buffer, 19 / sizeof(_read_buffer[0]), 0);

    if (sendResult == sizeof(_read_buffer))
    {
      // Wait for response
      std::memset(_recvbuf, 0, sizeof(_recvbuf));
      int bytesReceived = recv(sock, (char*) _recvbuf, sizeof(_recvbuf), 0);
      if (bytesReceived > 0)
      {
        if (_debug == true)
        {
          std::cout << "Bytes received: " << bytesReceived << std::endl;
          for (int i = 0; i < bytesReceived; i++)
          {
            // Echo response to console
            printf("%d ", _recvbuf[i]);
          }
          std::cout << std::endl;
        }

        int x = four_bytes_to_int(_recvbuf);
        return x;
      }
    }
    else
    {
      perror("Can't send telegram to D1, Err: ");
      exit(EXIT_FAILURE);
    }
  }

  float D1::get_si_unit_factor()
  {
    read_command(_READ_SI_UNIT_FACTOR, sizeof(_READ_SI_UNIT_FACTOR));
    // Read the SI Unit Position calculation of the multiplication factor when linear movement(byte 2 == 01h) is set;

    // for further informations please see manual chapter "Detailed description Motion Control Object" Object 60A8h and
    // Object 6092h
    if (_recvbuf[21] == 1)
    {
      // Equation to calculate the multiplication factor from the recieved byte 3 of object 60A8h
      if (_recvbuf[22] < 5)
      {
        float siUnitFactor = (std::pow(10, -3) / std::pow(10, _recvbuf[22]));
        return siUnitFactor;
      }
      else
      {
        float siUnitFactor = (std::pow(10, -3) / std::pow(10, _recvbuf[22] - 256));
        return siUnitFactor;
      }
    }
    // Read the SI Unit Positionand calculation of the multiplication factor when rotary movement(byte 2 == 41h) is set;
    // for further informations please see manual chapter "Detailed description Motion Control Object" Object 60A8h and
    // Object 6092h
    else if (_recvbuf[21] == 65)
    {
      // Equation to calculate the multiplication factor from the recieved byte 3 of object 60A8h
      if (_recvbuf[22] < 5)
      {
        float siUnitFactor = (1 / std::pow(10, _recvbuf[22]));
        return siUnitFactor;
      }
      else
      {
        float siUnitFactor = (1 / std::pow(10, _recvbuf[22] - 256));
        return siUnitFactor;
      }
    }
  }

  int D1::four_bytes_to_int(unsigned char data[])
  {
    // Conversion from 4 received bytes to integer
    int intValue;
    unsigned char buffer[4]{};
    buffer[0] = data[19];
    buffer[1] = data[20];
    buffer[2] = data[21];
    buffer[3] = data[22];
    memcpy(&intValue, buffer, sizeof(int));
    return intValue;
  }

  void D1::check_for_dryve_error()
  {
    read_command(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
    if (std::equal(std::begin(_STATUS_ERROR), std::end(_STATUS_ERROR), std::begin(_recvbuf)) ||
        std::equal(std::begin(_STATUS_ERROR_2), std::end(_STATUS_ERROR_2), std::begin(_recvbuf)) ||
        std::equal(std::begin(_STATUS_ERROR_3), std::end(_STATUS_ERROR_3), std::begin(_recvbuf)) ||
        std::equal(std::begin(_STATUS_ERROR_4), std::end(_STATUS_ERROR_4), std::begin(_recvbuf)))
    {
      // Read the Error Code and print it in the console
      int errorCode = read_object_value(0x60, 0x3f);
      std::string error;
      if (errorCode == 25376)
      {
        error = "E01 Error Configuration";
      }
      if (errorCode == 8992)
      {
        error = "E02 Motor Over-Current";
      }
      if (errorCode == 8977)
      {
        error = "E03 Encoder Over-Current";
      }
      if (errorCode == 8978)
      {
        error = "E04 10 V Output Over Current";
      }
      if (errorCode == 20756)
      {
        error = "E05 I/O Supply Low";
      }
      if (errorCode == 12834)
      {
        error = "E06 Logic Supply Low";
      }
      if (errorCode == 12562)
      {
        error = "E07 Logic Supply High";
      }
      if (errorCode == 12833)
      {
        error = "E08 Load Supply Low";
      }
      if (errorCode == 12817)
      {
        error = "E09 Load Supply High";
      }
      if (errorCode == 17168)
      {
        error = "E10 Temperature High";
      }
      if (errorCode == 34321)
      {
        error = "E11 Following Error";
      }
      if (errorCode == 65280)
      {
        error = "E12 Limit Switch";
      }
      if (errorCode == 29446)
      {
        error = "E13 Hall Sensor";
      }
      if (errorCode == 29445)
      {
        error = "E14 Encoder";
      }
      if (errorCode == 65281)
      {
        error = "E15 Encoder Channel A";
      }
      if (errorCode == 65282)
      {
        error = "E16 Encoder Channel B";
      }
      if (errorCode == 65283)
      {
        error = "E17 Encoder Channel I";
      }
      if (errorCode == 28944)
      {
        error = "E21 Braking Resistor Overload";
      }
      std::cerr << "ERROR: " << error << std::endl;
      exit(1);
    }
  }

  void D1::wait_for_ready()
  {
    do
    {
      read_command(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      check_for_dryve_error();
      if (_debug == true)
      {
        std::cout << "Waiting for the Movement to be finished!\n";
      }

    } while (std::equal(std::begin(_STATUS_READY), std::end(_STATUS_READY), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_READY_2), std::end(_STATUS_READY_2), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_READY_5), std::end(_STATUS_READY_5), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_READY_6), std::end(_STATUS_READY_6), std::begin(_recvbuf)) != true);
  }

  void D1::wait_for_homing()
  {
    do
    {
      read_command(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      check_for_dryve_error();
      if (_debug == true)
      {
        std::cout << "Waiting for the Homing to be finished!\n";
      }

    } while (std::equal(std::begin(_STATUS_READY), std::end(_STATUS_READY), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_READY_2), std::end(_STATUS_READY_2), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_READY_3), std::end(_STATUS_READY_3), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_READY_4), std::end(_STATUS_READY_4), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_READY_5), std::end(_STATUS_READY_5), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_READY_6), std::end(_STATUS_READY_6), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_READY_7), std::end(_STATUS_READY_7), std::begin(_recvbuf)) != true);
  }

  void D1::set_shutdown()
  {
    send_set_command(_SEND_SHUTDOWN, sizeof(_SEND_SHUTDOWN));
    do
    {
      read_command(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      if (_debug == true)
      {
        std::cout << "Waiting for the Shutdown!\n";
      }

    } while (std::equal(std::begin(_STATUS_SHUTDOWN), std::end(_STATUS_SHUTDOWN), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_SHUTDOWN_2), std::end(_STATUS_SHUTDOWN_2), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_SHUTDOWN_3), std::end(_STATUS_SHUTDOWN_3), std::begin(_recvbuf)) != true);
  }

  void D1::set_switch_on()
  {
    send_set_command(_SEND_SWITCH_ON, sizeof(_SEND_SWITCH_ON));
    do
    {
      read_command(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      if (_debug == true)
      {
        std::cout << "Waiting for Switch On!\n";
      }

    } while (std::equal(std::begin(_STATUS_SWITCH_ON), std::end(_STATUS_SWITCH_ON), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_SWITCH_ON_2), std::end(_STATUS_SWITCH_ON_2), std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_SWITCH_ON_3), std::end(_STATUS_SWITCH_ON_3), std::begin(_recvbuf)) != true);
  }

  void D1::set_operation_enable()
  {
    send_set_command(_SEND_OPERATION_ENABLE, sizeof(_SEND_OPERATION_ENABLE));
    do
    {
      read_command(_READ_STATUS_WORD, sizeof(_READ_STATUS_WORD));
      if (_debug == true)
      {
        std::cout << "Waiting for enable Operation!\n";
      }

    } while (std::equal(std::begin(_STATUS_OPERATION_ENABLE),
                        std::end(_STATUS_OPERATION_ENABLE),
                        std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_OPERATION_ENABLE_2),
                        std::end(_STATUS_OPERATION_ENABLE_2),
                        std::begin(_recvbuf)) != true &&
             std::equal(std::begin(_STATUS_OPERATION_ENABLE_3),
                        std::end(_STATUS_OPERATION_ENABLE_3),
                        std::begin(_recvbuf)) != true);
  }

  void D1::reset_status()
  {
    send_set_command(_RESET_DRYVE_STATUS, sizeof(_RESET_DRYVE_STATUS));
  }

  void D1::run_state_machine()

  {
    send_set_command(_send_reset_error, sizeof(_send_reset_error));
    send_set_command(_send_reset_array, sizeof(_send_reset_array));
    check_for_dryve_error();
    reset_status();
    set_shutdown();
    set_switch_on();
    set_operation_enable();
  }

  void D1::set_mode_of_operation(unsigned char mode)
  {
    unsigned char statusModeDisplay[] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode};
    _send_mode_of_operation[19] = mode;
    send_set_command(_send_mode_of_operation, sizeof(_send_mode_of_operation));
    do
    {
      read_command(_READ_MODES_DISPLAY, sizeof(_READ_MODES_DISPLAY));
      if (_debug == true)
      {
        std::cout << "Waiting for the Mode of Operation to be set! \n";
      }

    } while (std::equal(std::begin(statusModeDisplay), std::end(statusModeDisplay), std::begin(_recvbuf)) != true);
  }

  void D1::homing(float switchVelo, float zeroVelo, float homingAcc)
  {
    set_mode_of_operation(6);
    float siFactor = get_si_unit_factor();
    long switchVelocity = switchVelo * siFactor;
    long zeroVelocity = zeroVelo * siFactor;
    long homingAccel = homingAcc * siFactor;
    send_command(_send_switch_velocity, sizeof(_send_switch_velocity), switchVelocity);
    send_command(_send_zero_velocity, sizeof(_send_zero_velocity), zeroVelocity);
    send_command(_send_homing_acceleration, sizeof(_send_homing_acceleration), homingAccel);

    // Checks if the D1 is in an error state
    check_for_dryve_error();

    // Start Movement and toggle back bit 4
    send_set_command(_SEND_RESET_START, sizeof(_SEND_RESET_START));
    send_set_command(_SEND_START_MOVEMENT, sizeof(_SEND_START_MOVEMENT));

    // Wait for homing to end
    wait_for_homing();
  }

  void D1::profile_position_abs(float position, float velo, float accel, float decel)
  {
    set_mode_of_operation(1);
    float siFactor = get_si_unit_factor();
    long pos = position * siFactor;
    long velocity = velo * siFactor;
    long acc = accel * siFactor;
    long dec = decel * siFactor;
    send_command(_send_target_position, sizeof(_send_target_position), pos);
    send_command(_send_profile_velocity, sizeof(_send_profile_velocity), velocity);
    send_command(_send_profile_acceleration, sizeof(_send_profile_acceleration), acc);
    send_command(_send_profile_deceleration, sizeof(_send_profile_deceleration), dec);

    // Checks if the D1 is in an error state
    check_for_dryve_error();

    // Start Movement and toggle back bit 4
    send_set_command(_SEND_RESET_START_ABS, sizeof(_SEND_RESET_START_ABS));
    send_set_command(_SEND_START_MOVEMENT, sizeof(_SEND_START_MOVEMENT));

    // Wait for Movement to end
    std::cout << "Wait for Movement to end! \n";
    wait_for_ready();
    std::cout << "Movement ended! \n";
  }

  void D1::profile_position_rel(float position, float velo, float accel, float decel)
  {
    set_mode_of_operation(1);
    float siFactor = get_si_unit_factor();
    long pos = position * siFactor;
    long velocity = velo * siFactor;
    long acc = accel * siFactor;
    long dec = decel * siFactor;
    send_command(_send_target_position, sizeof(_send_target_position), pos);
    send_command(_send_profile_velocity, sizeof(_send_profile_velocity), velocity);
    send_command(_send_profile_acceleration, sizeof(_send_profile_acceleration), acc);
    send_command(_send_profile_deceleration, sizeof(_send_profile_deceleration), dec);

    // Checks if the D1 is in an error state
    check_for_dryve_error();

    // Start Movement and toggle back bit 4
    send_set_command(_SEND_RESET_START_REL, sizeof(_SEND_RESET_START_REL));
    send_set_command(_SEND_START_MOVEMENT_REL, sizeof(_SEND_START_MOVEMENT_REL));

    // Wait for Movement to end
    wait_for_ready();
  }

  void D1::profile_velocity(float velo, float accel, float decel)
  {
    set_mode_of_operation(3);
    float siFactor = get_si_unit_factor();
    long velocity = velo * siFactor;
    long acc = accel * siFactor;
    long dec = decel * siFactor;
    send_command(_send_profile_acceleration, sizeof(_send_profile_acceleration), acc);
    send_command(_send_profile_deceleration, sizeof(_send_profile_deceleration), dec);

    // Checks if the D1 is in an error state
    check_for_dryve_error();

    // Start movement by sending a target velocity value != 0 (0 for stop)
    send_command(_send_target_velocity, sizeof(_send_target_velocity), velocity);
  }

}   // namespace dryve_d1_gate
