#if !defined(DRYVE_D1_GATE__SOCKET_WRAPPER_HPP_)
#define DRYVE_D1_GATE__SOCKET_WRAPPER_HPP_

#include <sys/socket.h>

namespace dryve_d1_gate
{

  class ISocketWrapper
  {
   public:
    virtual ~ISocketWrapper() = default;
    virtual int receive(int fd, void *buf, size_t n, int flags) = 0;
  };

  class SocketWrapper : public ISocketWrapper
  {
   public:
    int receive(int fd, void *buf, size_t n, int flags) override
    {
      return recv(fd, buf, n, flags);
    }
  };

  class MockSocketWrapper : public ISocketWrapper
  {
   public:
    int receive(int fd, void *buf, size_t n, int flags) override
    {
      // return mock_recv(fd, buf, n, flags);
      // TODO: Mocking
    }

    // MOCK_METHOD(int, mock_recv, (int fd, void *buf, size_t n, int flags), ());
  };
}   // namespace dryve_d1_gate

#endif   // DRYVE_D1_GATE__SOCKET_WRAPPER_HPP_
