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
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
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
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
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
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
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
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
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
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
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
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
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
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
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
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
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

  SCENARIO("Testing the wait_for_dryve_ready_state function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      const unsigned char READ_STATUS_WORD[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2};
      const unsigned char STATUS_READY[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22};

      // Mock for first and second call of "_socket_wrapper->sending"
      EXPECT_CALL(*mock_socket_wrapper, sending(testing::_, testing::_, 19, 0))
          .WillOnce(testing::Return(19))
          .WillOnce(testing::Return(19));
      // Mock for first and second call of "_socket_wrapper->receiving"
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 23, 0))
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(STATUS_READY, STATUS_READY + sizeof(STATUS_READY)),
                                   testing::Return(sizeof(STATUS_READY))))
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(STATUS_READY, STATUS_READY + sizeof(STATUS_READY)),
                                   testing::Return(sizeof(STATUS_READY))));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("wait_for_dryve_ready_state is called")
      {
        std::string error_msg = d1.wait_for_dryve_ready_state();

        THEN(
            "the previously defined expected calls should run through, throw no error and the error msg should contain "
            "an empty string.")
        {
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
          REQUIRE(error_msg.empty() == true);
        }
      }
    }
  }

  SCENARIO("Testing the wait_for_homing function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      const unsigned char READ_STATUS_WORD[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2};
      const unsigned char STATUS_READY[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22};

      // Mock for first and second call of "_socket_wrapper->sending"
      EXPECT_CALL(*mock_socket_wrapper, sending(testing::_, testing::_, 19, 0))
          .WillOnce(testing::Return(19))
          .WillOnce(testing::Return(19));
      // Mock for first and second call of "_socket_wrapper->receiving"
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 23, 0))
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(STATUS_READY, STATUS_READY + sizeof(STATUS_READY)),
                                   testing::Return(sizeof(STATUS_READY))))
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(STATUS_READY, STATUS_READY + sizeof(STATUS_READY)),
                                   testing::Return(sizeof(STATUS_READY))));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("wait_for_homing is called")
      {
        std::string error_msg = d1.wait_for_homing();

        THEN(
            "the previously defined expected calls should run through, throw no error and the error msg should contain "
            "an empty string.")
        {
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
          REQUIRE(error_msg.empty() == true);
        }
      }
    }
  }

  SCENARIO("Testing the get_si_unit_factor function for a rotary movement of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls for a rotary movement")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      const unsigned char READ_SI_UNIT_FACTOR[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 168, 0, 0, 0, 0, 4};

      unsigned char expected_recv_buffer[23];
      expected_recv_buffer[19] = 0;
      expected_recv_buffer[20] = 0;
      expected_recv_buffer[21] = MOVEMENT_TYPE_ROTARY;
      expected_recv_buffer[22] = 0xFE;

      // Mock for first and second call of "_socket_wrapper->sending"
      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(READ_SI_UNIT_FACTOR) / sizeof(READ_SI_UNIT_FACTOR[0]), 0))
          .WillOnce(testing::Return(sizeof(READ_SI_UNIT_FACTOR)));
      // Mock for first and second call of "_socket_wrapper->receiving"
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 23, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(expected_recv_buffer, expected_recv_buffer + sizeof(expected_recv_buffer)),
              testing::Return(sizeof(expected_recv_buffer))));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("get_si_unit_factor is called")
      {
        float si_unit_factor = d1.get_si_unit_factor();

        THEN(
            "the previously defined expected calls should run through, throw no error and we should get the expected "
            "si unit factor")
        {
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
          REQUIRE(si_unit_factor == 100);
        }
      }
    }
  }

  SCENARIO("Testing the get_si_unit_factor function for a linear movement of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls for a linear movement")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      const unsigned char READ_SI_UNIT_FACTOR[19] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 168, 0, 0, 0, 0, 4};

      unsigned char expected_recv_buffer[23];
      expected_recv_buffer[19] = 0;
      expected_recv_buffer[20] = 0;
      expected_recv_buffer[21] = MOVEMENT_TYPE_LINEAR;
      expected_recv_buffer[22] = 0xFB;

      // Mock for first and second call of "_socket_wrapper->sending"
      EXPECT_CALL(*mock_socket_wrapper,
                  sending(testing::_, testing::_, sizeof(READ_SI_UNIT_FACTOR) / sizeof(READ_SI_UNIT_FACTOR[0]), 0))
          .WillOnce(testing::Return(sizeof(READ_SI_UNIT_FACTOR)));
      // Mock for first and second call of "_socket_wrapper->receiving"
      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 23, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(expected_recv_buffer, expected_recv_buffer + sizeof(expected_recv_buffer)),
              testing::Return(sizeof(expected_recv_buffer))));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("get_si_unit_factor is called")
      {
        float si_unit_factor = d1.get_si_unit_factor();

        THEN(
            "the previously defined expected calls should run through, throw no error and we should get the expected "
            "si unit factor")
        {
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
          REQUIRE(si_unit_factor == 100);
        }
      }
    }
  }

  SCENARIO("Testing the move_profile_to_absolute_position function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      float target_position = 10.0;
      float target_velocity = 5.0;
      float target_accel = 2.0;
      float target_decel = 1.0;

      // Preparation for set_dryve_mode_of_operation() func call
      unsigned char mode = 1;
      unsigned char send_mode_of_operation[20] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, mode};
      char handshake_send_mode_of_operation[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_mode_of_operation[12] = send_mode_of_operation[12];
      handshake_send_mode_of_operation[13] = send_mode_of_operation[13];
      handshake_send_mode_of_operation[14] = send_mode_of_operation[14];
      const unsigned char status_mode_display[] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode};

      // Preparation for get_si_unit_factor() func call
      unsigned char expected_recv_buffer[23];
      expected_recv_buffer[19] = 0;
      expected_recv_buffer[20] = 0;
      expected_recv_buffer[21] = MOVEMENT_TYPE_ROTARY;
      expected_recv_buffer[22] = 0xFE;

      // Preparation for check_for_dryve_error() func call
      const unsigned char STATUS_READY[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22};

      // Preparation for send_command_telegram(_send_target_position, ...) func call
      unsigned char send_target_position[23] = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                96, 122, 0, 0, 0, 0,  4, 0,  0,  0, 0};
      char handshake_send_target_position[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_target_position[12] = send_target_position[12];
      handshake_send_target_position[13] = send_target_position[13];
      handshake_send_target_position[14] = send_target_position[14];

      // Preparation for send_command_telegram(_send_profile_velocity, ...) func call
      unsigned char send_profile_velocity[23] = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                 96, 129, 0, 0, 0, 0,  4, 0,  0,  0, 0};
      char handshake_send_profile_velocity[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_profile_velocity[12] = send_profile_velocity[12];
      handshake_send_profile_velocity[13] = send_profile_velocity[13];
      handshake_send_profile_velocity[14] = send_profile_velocity[14];

      // Preparation for send_command_telegram(_send_profile_acceleration, ...) func
      // call
      unsigned char send_profile_acceleration[23] = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                     96, 131, 0, 0, 0, 0,  4, 0,  0,  0, 0};
      char handshake_send_profile_acceleration[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_profile_acceleration[12] = send_profile_acceleration[12];
      handshake_send_profile_acceleration[13] = send_profile_acceleration[13];
      handshake_send_profile_acceleration[14] = send_profile_acceleration[14];

      // Preparation for send_command_telegram(_send_profile_deceleration, ...) func call
      unsigned char send_profile_deceleration[23] = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                     96, 132, 0, 0, 0, 0,  4, 0,  0,  0, 0};
      char handshake_send_profile_deceleration[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_profile_deceleration[12] = send_profile_deceleration[12];
      handshake_send_profile_deceleration[13] = send_profile_deceleration[13];
      handshake_send_profile_deceleration[14] = send_profile_deceleration[14];

      // Preparation for send_command_telegram(_SEND_RESET_START_ABS, ...) func call
      const unsigned char SEND_RESET_START_ABS[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                      0, 96, 64, 0, 0, 0,  0, 2,  15, 0};
      char handshake_send_reset_start_abs[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_reset_start_abs[12] = SEND_RESET_START_ABS[12];
      handshake_send_reset_start_abs[13] = SEND_RESET_START_ABS[13];
      handshake_send_reset_start_abs[14] = SEND_RESET_START_ABS[14];

      // Preparation for send_command_telegram(_SEND_START_MOVEMENT, ...) func call
      const unsigned char SEND_START_MOVEMENT[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                     0, 96, 64, 0, 0, 0,  0, 2,  31, 0};
      char handshake_start_movement[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_start_movement[12] = SEND_START_MOVEMENT[12];
      handshake_start_movement[13] = SEND_START_MOVEMENT[13];
      handshake_start_movement[14] = SEND_START_MOVEMENT[14];

      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 19, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_send_mode_of_operation,
                                           handshake_send_mode_of_operation + sizeof(handshake_send_mode_of_operation)),
              testing::Return(sizeof(handshake_send_mode_of_operation))))   // mock for set_dryve_mode_of_operation(1)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_send_target_position,
                                           handshake_send_target_position + sizeof(handshake_send_target_position)),
              testing::Return(sizeof(
                  handshake_send_target_position))))   // mock for send_command_telegram(_send_target_position,...)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_send_profile_velocity,
                                           handshake_send_profile_velocity + sizeof(handshake_send_profile_velocity)),
              testing::Return(sizeof(
                  handshake_send_profile_velocity))))   // mock for send_command_telegram(_send_profile_velocity,...)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(
                  handshake_send_profile_acceleration,
                  handshake_send_profile_acceleration + sizeof(handshake_send_profile_acceleration)),
              testing::Return(sizeof(
                  handshake_send_profile_acceleration))))   // mock for
                                                            // send_command_telegram(_send_profile_acceleration,...)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(
                  handshake_send_profile_deceleration,
                  handshake_send_profile_deceleration + sizeof(handshake_send_profile_deceleration)),
              testing::Return(sizeof(
                  handshake_send_profile_deceleration))))   // mock for
                                                            // send_command_telegram(_send_profile_deceleration,...)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_send_reset_start_abs,
                                           handshake_send_reset_start_abs + sizeof(handshake_send_reset_start_abs)),
              testing::Return(sizeof(handshake_send_reset_start_abs))))   // mock for _SEND_RESET_START_ABS(1)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_start_movement,
                                           handshake_start_movement + sizeof(handshake_start_movement)),
              testing::Return(
                  sizeof(handshake_start_movement))));   // mock for send_command_telegram(_SEND_START_MOVEMENT,...)

      EXPECT_CALL(
          *mock_socket_wrapper,
          sending(testing::_, testing::_, sizeof(send_mode_of_operation) / sizeof(send_mode_of_operation[0]), 0))
          .WillOnce(testing::Return(sizeof(send_mode_of_operation)));

      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 23, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(status_mode_display, status_mode_display + sizeof(status_mode_display)),
              testing::Return(21)))   // mock for set_dryve_mode_of_operation(1)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(expected_recv_buffer, expected_recv_buffer + sizeof(expected_recv_buffer)),
              testing::Return(sizeof(expected_recv_buffer))))   // mock for get_si_unit_factor()
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(STATUS_READY, STATUS_READY + sizeof(STATUS_READY)),
                                   testing::Return(sizeof(STATUS_READY))))   // mock for check_for_dryve_error()
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(STATUS_READY, STATUS_READY + sizeof(STATUS_READY)),
                                   testing::Return(sizeof(STATUS_READY))))   // mock for wait_for_dryve_ready_state()
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(STATUS_READY, STATUS_READY + sizeof(STATUS_READY)),
                                   testing::Return(sizeof(STATUS_READY))));   // mock for wait_for_dryve_ready_state()

      EXPECT_CALL(*mock_socket_wrapper, sending(testing::_, testing::_, 19, 0))
          .WillOnce(testing::Return(19))    // mock for set_dryve_mode_of_operation(1)
          .WillOnce(testing::Return(19))    // mock for get_si_unit_factor()
          .WillOnce(testing::Return(19))    // mock for check_for_dryve_error()
          .WillOnce(testing::Return(19))    // mock for wait_for_dryve_ready_state()
          .WillOnce(testing::Return(19));   // mock for wait_for_dryve_ready_state()

      EXPECT_CALL(*mock_socket_wrapper, sending(testing::_, testing::_, 21, 0))
          .WillOnce(testing::Return(21))    // mock for send_command_telegram(_SEND_RESET_START_ABS,...)
          .WillOnce(testing::Return(21));   // mock for send_command_telegram(_SEND_START_MOVEMENT,...)

      EXPECT_CALL(*mock_socket_wrapper, sending(testing::_, testing::_, 23, 0))
          .WillOnce(testing::Return(23))    // mock for send_command_telegram(_send_target_position,...)
          .WillOnce(testing::Return(23))    // mock for send_command_telegram(_send_profile_velocity,...)
          .WillOnce(testing::Return(23))    // mock for send_command_telegram(_send_profile_acceleration,...)
          .WillOnce(testing::Return(23));   // mock for send_command_telegram(_send_profile_deceleration,...)

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("move_profile_to_absolute_position is called")
      {
        std::string error_msg =
            d1.move_profile_to_absolute_position(target_position, target_velocity, target_accel, target_decel);

        THEN(
            "the previously defined expected calls should run through, throw no error and we should get an empty error "
            "msg")
        {
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
          REQUIRE(error_msg.empty() == true);
        }
      }
    }
  }

  SCENARIO("Testing the move_profile_to_relative_position function of the D1 class")
  {
    GIVEN("A test server, the D1 class and the expected calls")
    {
      int port = 12345;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      std::unique_ptr<MockSocketWrapper> mock_socket_wrapper = std::make_unique<MockSocketWrapper>();

      float target_position = 10.0;
      float target_velocity = 5.0;
      float target_accel = 2.0;
      float target_decel = 1.0;

      // Preparation for set_dryve_mode_of_operation() func call
      unsigned char mode = 1;
      unsigned char send_mode_of_operation[20] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, mode};
      char handshake_send_mode_of_operation[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_mode_of_operation[12] = send_mode_of_operation[12];
      handshake_send_mode_of_operation[13] = send_mode_of_operation[13];
      handshake_send_mode_of_operation[14] = send_mode_of_operation[14];
      const unsigned char status_mode_display[] = {0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode};

      // Preparation for get_si_unit_factor() func call
      unsigned char expected_recv_buffer[23];
      expected_recv_buffer[19] = 0;
      expected_recv_buffer[20] = 0;
      expected_recv_buffer[21] = MOVEMENT_TYPE_ROTARY;
      expected_recv_buffer[22] = 0xFE;

      // Preparation for check_for_dryve_error() func call
      const unsigned char STATUS_READY[21] = {0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22};

      // Preparation for send_command_telegram(_send_target_position, ...) func call
      unsigned char send_target_position[23] = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                96, 122, 0, 0, 0, 0,  4, 0,  0,  0, 0};
      char handshake_send_target_position[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_target_position[12] = send_target_position[12];
      handshake_send_target_position[13] = send_target_position[13];
      handshake_send_target_position[14] = send_target_position[14];

      // Preparation for send_command_telegram(_send_profile_velocity, ...) func call
      unsigned char send_profile_velocity[23] = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                 96, 129, 0, 0, 0, 0,  4, 0,  0,  0, 0};
      char handshake_send_profile_velocity[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_profile_velocity[12] = send_profile_velocity[12];
      handshake_send_profile_velocity[13] = send_profile_velocity[13];
      handshake_send_profile_velocity[14] = send_profile_velocity[14];

      // Preparation for send_command_telegram(_send_profile_acceleration, ...) func
      // call
      unsigned char send_profile_acceleration[23] = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                     96, 131, 0, 0, 0, 0,  4, 0,  0,  0, 0};
      char handshake_send_profile_acceleration[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_profile_acceleration[12] = send_profile_acceleration[12];
      handshake_send_profile_acceleration[13] = send_profile_acceleration[13];
      handshake_send_profile_acceleration[14] = send_profile_acceleration[14];

      // Preparation for send_command_telegram(_send_profile_deceleration, ...) func call
      unsigned char send_profile_deceleration[23] = {0,  0,   0, 0, 0, 17, 0, 43, 13, 1, 0, 0,
                                                     96, 132, 0, 0, 0, 0,  4, 0,  0,  0, 0};
      char handshake_send_profile_deceleration[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_profile_deceleration[12] = send_profile_deceleration[12];
      handshake_send_profile_deceleration[13] = send_profile_deceleration[13];
      handshake_send_profile_deceleration[14] = send_profile_deceleration[14];

      // Preparation for send_command_telegram(_SEND_RESET_START_REL, ...) func call
      const unsigned char SEND_RESET_START_REL[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                      0, 96, 64, 0, 0, 0,  0, 2,  79, 0};
      char handshake_send_reset_start_rel[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_send_reset_start_rel[12] = SEND_RESET_START_REL[12];
      handshake_send_reset_start_rel[13] = SEND_RESET_START_REL[13];
      handshake_send_reset_start_rel[14] = SEND_RESET_START_REL[14];

      // Preparation for send_command_telegram(_SEND_START_MOVEMENT, ...) func call
      const unsigned char SEND_START_MOVEMENT[21] = {0, 0,  0,  0, 0, 15, 0, 43, 13, 1, 0,
                                                     0, 96, 64, 0, 0, 0,  0, 2,  31, 0};
      char handshake_start_movement[] = {0, 0, 0, 0, 0, 13, 0, 43, 13, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      handshake_start_movement[12] = SEND_START_MOVEMENT[12];
      handshake_start_movement[13] = SEND_START_MOVEMENT[13];
      handshake_start_movement[14] = SEND_START_MOVEMENT[14];

      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 19, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_send_mode_of_operation,
                                           handshake_send_mode_of_operation + sizeof(handshake_send_mode_of_operation)),
              testing::Return(sizeof(handshake_send_mode_of_operation))))   // mock for set_dryve_mode_of_operation(1)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_send_target_position,
                                           handshake_send_target_position + sizeof(handshake_send_target_position)),
              testing::Return(sizeof(
                  handshake_send_target_position))))   // mock for send_command_telegram(_send_target_position,...)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_send_profile_velocity,
                                           handshake_send_profile_velocity + sizeof(handshake_send_profile_velocity)),
              testing::Return(sizeof(
                  handshake_send_profile_velocity))))   // mock for send_command_telegram(_send_profile_velocity,...)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(
                  handshake_send_profile_acceleration,
                  handshake_send_profile_acceleration + sizeof(handshake_send_profile_acceleration)),
              testing::Return(sizeof(
                  handshake_send_profile_acceleration))))   // mock for
                                                            // send_command_telegram(_send_profile_acceleration,...)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(
                  handshake_send_profile_deceleration,
                  handshake_send_profile_deceleration + sizeof(handshake_send_profile_deceleration)),
              testing::Return(sizeof(
                  handshake_send_profile_deceleration))))   // mock for
                                                            // send_command_telegram(_send_profile_deceleration,...)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_send_reset_start_rel,
                                           handshake_send_reset_start_rel + sizeof(handshake_send_reset_start_rel)),
              testing::Return(sizeof(handshake_send_reset_start_rel))))   // mock for _SEND_RESET_START_REL(1)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(handshake_start_movement,
                                           handshake_start_movement + sizeof(handshake_start_movement)),
              testing::Return(
                  sizeof(handshake_start_movement))));   // mock for send_command_telegram(_SEND_START_MOVEMENT,...)

      EXPECT_CALL(
          *mock_socket_wrapper,
          sending(testing::_, testing::_, sizeof(send_mode_of_operation) / sizeof(send_mode_of_operation[0]), 0))
          .WillOnce(testing::Return(sizeof(send_mode_of_operation)));

      EXPECT_CALL(*mock_socket_wrapper, receiving(testing::_, testing::_, 23, 0))
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(status_mode_display, status_mode_display + sizeof(status_mode_display)),
              testing::Return(21)))   // mock for set_dryve_mode_of_operation(1)
          .WillOnce(testing::DoAll(
              testing::SetArrayArgument<1>(expected_recv_buffer, expected_recv_buffer + sizeof(expected_recv_buffer)),
              testing::Return(sizeof(expected_recv_buffer))))   // mock for get_si_unit_factor()
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(STATUS_READY, STATUS_READY + sizeof(STATUS_READY)),
                                   testing::Return(sizeof(STATUS_READY))))   // mock for check_for_dryve_error()
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(STATUS_READY, STATUS_READY + sizeof(STATUS_READY)),
                                   testing::Return(sizeof(STATUS_READY))))   // mock for wait_for_dryve_ready_state()
          .WillOnce(testing::DoAll(testing::SetArrayArgument<1>(STATUS_READY, STATUS_READY + sizeof(STATUS_READY)),
                                   testing::Return(sizeof(STATUS_READY))));   // mock for wait_for_dryve_ready_state()

      EXPECT_CALL(*mock_socket_wrapper, sending(testing::_, testing::_, 19, 0))
          .WillOnce(testing::Return(19))    // mock for set_dryve_mode_of_operation(1)
          .WillOnce(testing::Return(19))    // mock for get_si_unit_factor()
          .WillOnce(testing::Return(19))    // mock for check_for_dryve_error()
          .WillOnce(testing::Return(19))    // mock for wait_for_dryve_ready_state()
          .WillOnce(testing::Return(19));   // mock for wait_for_dryve_ready_state()

      EXPECT_CALL(*mock_socket_wrapper, sending(testing::_, testing::_, 21, 0))
          .WillOnce(testing::Return(21))    // mock for send_command_telegram(_SEND_RESET_START_REL,...)
          .WillOnce(testing::Return(21));   // mock for send_command_telegram(_SEND_START_MOVEMENT,...)

      EXPECT_CALL(*mock_socket_wrapper, sending(testing::_, testing::_, 23, 0))
          .WillOnce(testing::Return(23))    // mock for send_command_telegram(_send_target_position,...)
          .WillOnce(testing::Return(23))    // mock for send_command_telegram(_send_profile_velocity,...)
          .WillOnce(testing::Return(23))    // mock for send_command_telegram(_send_profile_acceleration,...)
          .WillOnce(testing::Return(23));   // mock for send_command_telegram(_send_profile_deceleration,...)

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("move_profile_to_relative_position is called")
      {
        std::string error_msg =
            d1.move_profile_to_relative_position(target_position, target_velocity, target_accel, target_decel);

        THEN(
            "the previously defined expected calls should run through, throw no error and we should get an empty error "
            "msg")
        {
          REQUIRE(testing::Mock::VerifyAndClearExpectations(&mock_socket_wrapper));
          REQUIRE(error_msg.empty() == true);
        }
      }
    }
  }

}   // namespace unit_test
