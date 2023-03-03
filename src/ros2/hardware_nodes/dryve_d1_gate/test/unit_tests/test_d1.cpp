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

  SCENARIO("Testing the functionality of the D1 class")
  {
    GIVEN("A test server")
    {
      int port = 123;
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

    GIVEN("A test server and the D1 class")
    {
      int port = 1234;
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
                             testing::Return(23)));

      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::move(mock_socket_wrapper));

      WHEN("set_dryve_shutdown_state is called")
      {
        d1.set_dryve_shutdown_state();

        THEN("the previously defined expected calls should run through and throw no error")
        {
          // There is no check we can do here as we only want the set_dryve_shutdown_state to run through with our
          // predefined expected calls
          REQUIRE(true);
        }
      }
    }
  }

}   // namespace unit_test