#if !defined(DRYVE_D1_GATE__MOCK_SOCKET_WRAPPER_HPP_)
#define DRYVE_D1_GATE__MOCK_SOCKET_WRAPPER_HPP_

#include "dryve_d1_gate/i_socket_wrapper.hpp"
#include "gmock/gmock.h"

namespace unit_test
{
  class MockSocketWrapper : public dryve_d1_gate::ISocketWrapper
  {
   public:
    // int receiving(int fd, void *buf, size_t n, int flags) override
    // {
    //   return mock_receive(fd, buf, n, flags);
    // }
    // unsigned int sending(int fd, const void *buf, size_t n, int flags) override
    // {
    //   return mock_send(fd, buf, n, flags);
    // }

    // MOCK_METHOD(int, mock_receive, (int fd, void *buf, size_t n, int flags), ());
    // MOCK_METHOD(int, mock_send, (int fd, const void *buf, size_t n, int flags), ());

    MOCK_METHOD(int, receiving, (int fd, void *buf, size_t n, int flags), (override));
    MOCK_METHOD(unsigned int, sending, (int fd, const void *buf, size_t n, int flags), (override));
  };
}   // namespace unit_test

#endif   // DRYVE_D1_GATE__MOCK_SOCKET_WRAPPER_HPP_
