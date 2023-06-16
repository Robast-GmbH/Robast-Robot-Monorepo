#include "dryve_d1_gate/d1.hpp"
#include "dryve_d1_gate/socket_wrapper.hpp"

int main(int argc, char** argv)
{
  const std::string DRYVE_D1_IP_ADDRESS = "10.10.13.5";
  const int PORT = 502;

  auto d1 =
      std::make_unique<dryve_d1_gate::D1>(DRYVE_D1_IP_ADDRESS, PORT, std::make_unique<dryve_d1_gate::SocketWrapper>());

  d1->wait_for_dryve_ready_state();
  d1->set_dryve_shutdown_state();

  close(d1->sock);

  return 0;
}
