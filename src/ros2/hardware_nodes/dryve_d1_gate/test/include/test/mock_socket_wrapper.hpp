#if !defined(DRYVE_D1_GATE__MOCK_SOCKET_WRAPPER_HPP_)
#define DRYVE_D1_GATE__MOCK_SOCKET_WRAPPER_HPP_

#include "dryve_d1_gate/i_socket_wrapper.hpp"
#include "gmock/gmock.h"

namespace unit_test
{
  class MockSocketWrapper : public dryve_d1_gate::ISocketWrapper
  {
   public:
    MOCK_METHOD(int, receiving, (int fd, char *buf, size_t n, int flags), (override));
    MOCK_METHOD(unsigned int, sending, (int fd, const void *buf, size_t n, int flags), (override));
  };
}   // namespace unit_test

#endif   // DRYVE_D1_GATE__MOCK_SOCKET_WRAPPER_HPP_
