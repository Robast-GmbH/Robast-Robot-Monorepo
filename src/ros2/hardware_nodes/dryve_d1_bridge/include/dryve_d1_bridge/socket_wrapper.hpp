#ifndef DRYVE_D1_BRIDGE__SOCKET_WRAPPER_HPP_
#define DRYVE_D1_BRIDGE__SOCKET_WRAPPER_HPP_

#include <sys/socket.h>

#include "dryve_d1_bridge/i_socket_wrapper.hpp"

namespace dryve_d1_bridge
{
  class SocketWrapper : public ISocketWrapper
  {
   public:
    int receiving(int fd, char *buf, size_t n, int flags) override
    {
      return recv(fd, buf, n, flags);
    }

    unsigned int sending(int fd, const void *buf, size_t n, int flags) override
    {
      return send(fd, buf, n, flags);
    }
  };
}   // namespace dryve_d1_bridge

#endif   // DRYVE_D1_BRIDGE__I_SOCKET_WRAPPER_HPP_
