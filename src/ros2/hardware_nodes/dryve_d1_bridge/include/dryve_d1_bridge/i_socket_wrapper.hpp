#ifndef DRYVE_D1_BRIDGE__I_SOCKET_WRAPPER_HPP_
#define DRYVE_D1_BRIDGE__I_SOCKET_WRAPPER_HPP_

namespace dryve_d1_bridge
{
  class ISocketWrapper
  {
   public:
    virtual ~ISocketWrapper() = default;
    virtual int receiving(int fd, char *buf, size_t n, int flags) = 0;
    virtual unsigned int sending(int fd, const void *buf, size_t n, int flags) = 0;
  };

}   // namespace dryve_d1_bridge

#endif   // DRYVE_D1_BRIDGE__I_SOCKET_WRAPPER_HPP_
