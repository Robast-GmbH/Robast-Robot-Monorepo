#include "dryve_d1_gate/d1.hpp"
#include "dryve_d1_gate/d1_configs.hpp"
#include "dryve_d1_gate/socket_wrapper.hpp"

int main(int argc, char** argv)
{
  std::unique_ptr<dryve_d1_gate::D1> d1_x_axis = std::make_unique<dryve_d1_gate::D1>(
      dryve_d1_gate::DRYVE_D1_IP_ADDRESS_X_AXIS, dryve_d1_gate::PORT, std::make_unique<dryve_d1_gate::SocketWrapper>());
  std::unique_ptr<dryve_d1_gate::D1> d1_y_axis = std::make_unique<dryve_d1_gate::D1>(
      dryve_d1_gate::DRYVE_D1_IP_ADDRESS_Y_AXIS, dryve_d1_gate::PORT, std::make_unique<dryve_d1_gate::SocketWrapper>());

  d1_x_axis->close_connection();
  d1_y_axis->close_connection();

  return 0;
}
