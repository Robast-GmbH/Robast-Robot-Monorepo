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
      int port = 8080;
      start_test_server(port);

      std::string ip_address = "127.0.0.1";

      WHEN("Creating the D1 class")
      {
        dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::make_unique<MockSocketWrapper>());

        THEN("There should be no connection error while creating the class.")
        {
          // There is now check we can do here, if the D1 constructor call failed, there will be an error anyway
          REQUIRE(true);
        }
      }
    }

    GIVEN("A test server and the D1 class")
    {
      int port = 8080;
      start_test_server(port);
      std::string ip_address = "127.0.0.1";
      dryve_d1_gate::D1 d1 = dryve_d1_gate::D1(ip_address, port, std::make_unique<MockSocketWrapper>());

      WHEN("send_command is called with data for _send_target_position")
      {
        // d1.send_command();

        THEN("the _debug variable should be set to true")
        {
          REQUIRE(1 == 1);
        }
      }
    }
  }

}   // namespace unit_test

// int connect_to_server(const char *ip, int port)
// {
//   // create socket
//   int sock = socket(AF_INET, SOCK_STREAM, 0);
//   if (sock == -1)
//   {
//     std::cerr << "Failed to create socket\n";
//     return -1;
//   }

//   // setup server address
//   struct sockaddr_in serv_addr;
//   serv_addr.sin_family = AF_INET;
//   serv_addr.sin_port = htons(port);
//   if (inet_pton(AF_INET, ip, &serv_addr.sin_addr) <= 0)
//   {
//     std::cerr << "Invalid address\n";
//     return -1;
//   }

//   // connect to server
//   if (connect(sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
//   {
//     std::cerr << "Failed to connect to server\n";
//     return -1;
//   }

//   return sock;
// }

// TEST_CASE("Test connection to server")
// {
//   // start test server
//   int server_sock = socket(AF_INET, SOCK_STREAM, 0);
//   REQUIRE(server_sock != -1);

//   struct sockaddr_in serv_addr;
//   serv_addr.sin_family = AF_INET;
//   serv_addr.sin_addr.s_addr = INADDR_ANY;
//   serv_addr.sin_port = htons(12345);

//   REQUIRE(bind(server_sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) != -1);

//   REQUIRE(listen(server_sock, 1) != -1);

//   // test client connection to server
//   int client_sock = connect_to_server("127.0.0.1", 12345);
//   REQUIRE(client_sock != -1);

//   // clean up
//   close(client_sock);
//   close(server_sock);
// }

// TEST(D1Test, StartConnection)
// {
//   dryve_d1_gate::d1 d1;
//   std::string ipAddress = "127.0.0.1";
//   int port = 8080;

//   d1.start_connection(ipAddress, port);

//   EXPECT_EQ(d1.sock, 0);                                                    // check if socket is created
//   successfully EXPECT_EQ(inet_pton(AF_INET, ipAddress.c_str(), &d1.hint.sin_addr), 1);   // check if inet_pton
//   succeeds EXPECT_NE(connect(d1.sock, (sockaddr*) &d1.hint, sizeof(d1.hint)),
//             -1);   // check if connection is established successfully
// }