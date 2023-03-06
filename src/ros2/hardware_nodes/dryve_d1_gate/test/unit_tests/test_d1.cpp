#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cassert>
#include <catch2/catch_all.hpp>
#include <iostream>

#include "dryve_d1_gate/d1.hpp"
#include "test/mock_socket_wrapper.hpp"

namespace unit_test
{
  void start_test_server(int port)
  {
    // start test server
    int server_sock = socket(AF_INET, SOCK_STREAM, 0);
    REQUIRE(server_sock != -1);

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);

    REQUIRE(bind(server_sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) != -1);

    REQUIRE(listen(server_sock, 1) != -1);
  }

  SCENARIO("Testing the Creating of the D1 class")
  {
    GIVEN("A test server")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";

      WHEN("Creating the D1 class")
      {
        dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::make_unique<MockSocketWrapper>());

        THEN("There should be no connection error while creating the class.")
        {
          // There is no check we can do here, if the D1 constructor call failed, there will be an error anyway
          REQUIRE(true);
        }
      }
    }
  }

  SCENARIO("Testing the set_dryve_shutdown_state function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      const unsigned char SEND_SHUTDOWN[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 6, 0};
      char handshake_send_shutdown[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_shutdown[12] = SEND_SHUTDOWN[12];
      handshake_send_shutdown[13] = SEND_SHUTDOWN[13];
      handshake_send_shutdown[14] = SEND_SHUTDOWN[14];

      const unsigned char READ_STATUS_WORD[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2};

      const unsigned char STATUS_SHUTDOWN[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 6};

      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(SEND_SHUTDOWN) / sizeof(SEND_SHUTDOWN[0]), 0))
          .WillOnce(testing::Return(sizeof(SEND_SHUTDOWN)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 19, 0))
          .WillOnce(
              testing::DoAll(testing::SetArrayArgument<1>(handshake_send_shutdown,
                                                          handshake_send_shutdown + sizeof(handshake_send_shutdown)),
                             testing::Return(sizeof(handshake_send_shutdown))));

      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(READ_STATUS_WORD) / sizeof(READ_STATUS_WORD[0]), 0))
          .WillOnce(testing::Return(sizeof(READ_STATUS_WORD)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 23, 0))
          .WillOnce(
              testing::DoAll(testing::SetArrayArgument<1>(STATUS_SHUTDOWN, STATUS_SHUTDOWN + sizeof(STATUS_SHUTDOWN)),
                             testing::Return(21)));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("set_dryve_shutdown_state is called")
      {
        d1.set_dryve_shutdown_state();

        THEN("the previously defined expected calls should run through and throw no error")
        {
          // There is no check we can do here as we only want the called function to run through with our
          // predefined expected calls
          REQUIRE(true);
        }
      }
    }
  }

  SCENARIO("Testing the set_dryve_switch_on_state function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      const unsigned char SEND_SWITCH_ON[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 7, 0};
      char handshake_send_switch_on[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_switch_on[12] = SEND_SWITCH_ON[12];
      handshake_send_switch_on[13] = SEND_SWITCH_ON[13];
      handshake_send_switch_on[14] = SEND_SWITCH_ON[14];

      const unsigned char READ_STATUS_WORD[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2};

      const unsigned char STATUS_SWITCH_ON[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 6};

      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(SEND_SWITCH_ON) / sizeof(SEND_SWITCH_ON[0]), 0))
          .WillOnce(testing::Return(sizeof(SEND_SWITCH_ON)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 19, 0))
          .WillOnce(
              testing::DoAll(testing::SetArrayArgument<1>(handshake_send_switch_on,
                                                          handshake_send_switch_on + sizeof(handshake_send_switch_on)),
                             testing::Return(sizeof(handshake_send_switch_on))));

      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(READ_STATUS_WORD) / sizeof(READ_STATUS_WORD[0]), 0))
          .WillOnce(testing::Return(sizeof(READ_STATUS_WORD)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 23, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(STATUS_SWITCH_ON, STATUS_SWITCH_ON + sizeof(STATUS_SWITCH_ON)),
              testing::Return(21)));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("set_dryve_switch_on_state is called")
      {
        d1.set_dryve_switch_on_state();

        THEN("the previously defined expected calls should run through and throw no error")
        {
          // There is no check we can do here as we only want the called function to run through with our
          // predefined expected calls
          REQUIRE(true);
        }
      }
    }
  }

  SCENARIO("Testing the set_dryve_operation_enable_state function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      const unsigned char SEND_OPERATION_ENABLE[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                       0, 96, 64, 0, 0, 0,  0, 2,  15, 0};
      char handshake_send_operation_enable[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_operation_enable[12] = SEND_OPERATION_ENABLE[12];
      handshake_send_operation_enable[13] = SEND_OPERATION_ENABLE[13];
      handshake_send_operation_enable[14] = SEND_OPERATION_ENABLE[14];

      const unsigned char READ_STATUS_WORD[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2};

      const unsigned char STATUS_OPERATION_ENABLE[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                         0, 96, 65, 0, 0, 0,  0, 2,  39, 6};
      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(SEND_OPERATION_ENABLE) / sizeof(SEND_OPERATION_ENABLE[0]), 0))
          .WillOnce(testing::Return(sizeof(SEND_OPERATION_ENABLE)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 19, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_send_operation_enable,
                                           handshake_send_operation_enable + sizeof(handshake_send_operation_enable)),
              testing::Return(sizeof(handshake_send_operation_enable))));

      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(READ_STATUS_WORD) / sizeof(READ_STATUS_WORD[0]), 0))
          .WillOnce(testing::Return(sizeof(READ_STATUS_WORD)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 23, 0))
          .WillOnce(
              testing::DoAll(testing::SetArrayArgument<1>(STATUS_OPERATION_ENABLE,
                                                          STATUS_OPERATION_ENABLE + sizeof(STATUS_OPERATION_ENABLE)),
                             testing::Return(21)));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("set_dryve_operation_enable_state is called")
      {
        d1.set_dryve_operation_enable_state();

        THEN("the previously defined expected calls should run through and throw no error")
        {
          // There is no check we can do here as we only want the called function to run through with our
          // predefined expected calls
          REQUIRE(true);
        }
      }
    }
  }

  SCENARIO("Testing the send_constant_set_command function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      const unsigned char SEND_OPERATION_ENABLE[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                       0, 96, 64, 0, 0, 0,  0, 2,  15, 0};
      char handshake_send_operation_enable[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_operation_enable[12] = SEND_OPERATION_ENABLE[12];
      handshake_send_operation_enable[13] = SEND_OPERATION_ENABLE[13];
      handshake_send_operation_enable[14] = SEND_OPERATION_ENABLE[14];

      const unsigned char STATUS_OPERATION_ENABLE[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                         0, 96, 65, 0, 0, 0,  0, 2,  39, 6};
      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(SEND_OPERATION_ENABLE) / sizeof(SEND_OPERATION_ENABLE[0]), 0))
          .WillOnce(testing::Return(sizeof(SEND_OPERATION_ENABLE)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 19, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_send_operation_enable,
                                           handshake_send_operation_enable + sizeof(handshake_send_operation_enable)),
              testing::Return(sizeof(handshake_send_operation_enable))));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("send_constant_set_command is called")
      {
        d1.send_constant_set_command(SEND_OPERATION_ENABLE, sizeof(SEND_OPERATION_ENABLE));

        THEN("the previously defined expected calls should run through and throw no error")
        {
          // There is no check we can do here as we only want the called function to run through with our
          // predefined expected calls
          REQUIRE(true);
        }
      }
    }
  }

  SCENARIO("Testing the reset_dryve_status function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      const unsigned char RESET_DRYVE_STATUS[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 0, 1};
      char handshake_reset_dryve_status[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_reset_dryve_status[12] = RESET_DRYVE_STATUS[12];
      handshake_reset_dryve_status[13] = RESET_DRYVE_STATUS[13];
      handshake_reset_dryve_status[14] = RESET_DRYVE_STATUS[14];

      const unsigned char STATUS_OPERATION_ENABLE[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 0, 0,
                                                         0, 96, 65, 0, 0, 0,  0, 2,  39, 6};
      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(RESET_DRYVE_STATUS) / sizeof(RESET_DRYVE_STATUS[0]), 0))
          .WillOnce(testing::Return(sizeof(RESET_DRYVE_STATUS)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 19, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_reset_dryve_status,
                                           handshake_reset_dryve_status + sizeof(handshake_reset_dryve_status)),
              testing::Return(sizeof(handshake_reset_dryve_status))));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("reset_dryve_status is called")
      {
        d1.reset_dryve_status();

        THEN("the previously defined expected calls should run through and throw no error")
        {
          // There is no check we can do here as we only want the called function to run through with our
          // predefined expected calls
          REQUIRE(true);
        }
      }
    }
  }

  SCENARIO("Testing the set_dryve_mode_of_operation function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      unsigned char mode = 6;

      unsigned char send_mode_of_operation[20] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, mode};

      char handshake_send_mode_of_operation[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_mode_of_operation[12] = send_mode_of_operation[12];
      handshake_send_mode_of_operation[13] = send_mode_of_operation[13];
      handshake_send_mode_of_operation[14] = send_mode_of_operation[14];

      const unsigned char READ_MODES_DISPLAY[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1};

      unsigned char status_mode_display[] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode};

      EXPECT_CALL(
          *mock_socket_wrapper,
          sending(testing::_, testing::_, sizeof(send_mode_of_operation) / sizeof(send_mode_of_operation[0]), 0))
          .WillOnce(testing::Return(sizeof(send_mode_of_operation)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 19, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_send_mode_of_operation,
                                           handshake_send_mode_of_operation + sizeof(handshake_send_mode_of_operation)),
              testing::Return(sizeof(handshake_send_mode_of_operation))));

      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(READ_MODES_DISPLAY) / sizeof(READ_MODES_DISPLAY[0]), 0))
          .WillOnce(testing::Return(sizeof(READ_MODES_DISPLAY)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 23, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(status_mode_display, status_mode_display + sizeof(status_mode_display)),
              testing::Return(21)));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("set_dryve_mode_of_operation is called")
      {
        d1.set_dryve_mode_of_operation(mode);

        THEN("the previously defined expected calls should run through and throw no error")
        {
          // There is no check we can do here as we only want the called function to run through with our
          // predefined expected calls
          REQUIRE(true);
        }
      }
    }
  }

  SCENARIO("Testing the read_command_to_recv_buffer function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      const unsigned char SEND_SHUTDOWN[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 6, 0};
      unsigned char recv_buffer[23];

      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(SEND_SHUTDOWN) / sizeof(SEND_SHUTDOWN[0]), 0))
          .WillOnce(testing::Return(sizeof(SEND_SHUTDOWN)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, sizeof(recv_buffer), 0))
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(recv_buffer, recv_buffer + sizeof(recv_buffer)),
                                   testing::Return(sizeof(recv_buffer))));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("read_command_to_recv_buffer is called")
      {
        d1.read_command_to_recv_buffer(SEND_SHUTDOWN, sizeof(SEND_SHUTDOWN));

        THEN("the previously defined expected calls should run through and throw no error")
        {
          // There is no check we can do here as we only want the called function to run through with our
          // predefined expected calls
          REQUIRE(true);
        }
      }
    }
  }

  SCENARIO("Testing the read_object_value function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      char object_index_1 = 0x60;
      char object_index_2 = 0x64;
      int subindex = 0;

      unsigned char read_buffer[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};
      read_buffer[12] = int(object_index_1);
      read_buffer[13] = int(object_index_2);
      read_buffer[14] = subindex;

      unsigned char expected_recv_buffer[23];
      expected_recv_buffer[19] = 1;
      expected_recv_buffer[20] = 2;
      expected_recv_buffer[21] = 3;
      expected_recv_buffer[22] = 4;
      int expected_int_value = 0x04030201;

      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(read_buffer) / sizeof(read_buffer[0]), 0))
          .WillOnce(testing::Return(sizeof(read_buffer)));
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, sizeof(expected_recv_buffer), 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(expected_recv_buffer, expected_recv_buffer + sizeof(expected_recv_buffer)),
              testing::Return(sizeof(expected_recv_buffer))));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("read_object_value is called")
      {
        // Read the value of the Object 6064.0 "Position Actual Value"
        int object_value = d1.read_object_value(object_index_1, object_index_2, subindex);

        THEN(
            "the previously defined expected calls should run through and throw no error and the function should "
            "return the read object value")
        {
          REQUIRE(object_value == expected_int_value);
        }
      }
    }
  }

  SCENARIO("Testing the check_for_dryve_error function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      const unsigned char READ_STATUS_WORD[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2};
      const unsigned char STATUS_ERROR[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 6};

      char object_index_1 = 0x60;
      char object_index_2 = 0x3f;
      int subindex = 0;

      unsigned char read_buffer[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};
      read_buffer[12] = int(object_index_1);
      read_buffer[13] = int(object_index_2);
      read_buffer[14] = subindex;

      unsigned char expected_recv_buffer[23];
      expected_recv_buffer[19] = 0x20;
      expected_recv_buffer[20] = 0x63;
      expected_recv_buffer[21] = 0;
      expected_recv_buffer[22] = 0;

      // Mock for first and second call of "_socket_wrapper->sending"
      EXPECT_CALL(*mock_socket_wrapper, sending(testing::_, testing::_, 19, 0))
          .WillOnce(testing::Return(19))
          .WillOnce(testing::Return(19));
      // Mock for first and second call of "_socket_wrapper->receiving"
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 23, 0))
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(STATUS_ERROR, STATUS_ERROR + sizeof(STATUS_ERROR)),
                                   testing::Return(sizeof(STATUS_ERROR))))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(expected_recv_buffer, expected_recv_buffer + sizeof(expected_recv_buffer)),
              testing::Return(sizeof(expected_recv_buffer))));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("check_for_dryve_error is called")
      {
        std::string error_msg = d1.check_for_dryve_error();

        THEN(
            "the previously defined expected calls should run through, throw no error and we should get the expected "
            "error message")
        {
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
          REQUIRE(error_msg == "E01 Error Configuration");
        }
      }
    }
  }

}   // namespace unit_test
