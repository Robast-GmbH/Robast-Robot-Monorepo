#include "dryve_d1_bridge/d1.hpp"
#include "dryve_d1_bridge/d1_configs.hpp"
#include "dryve_d1_bridge/socket_wrapper.hpp"

int main(int argc, char** argv)
{
  std::unique_ptr<dryve_d1_bridge::D1> d1_x_axis =
      std::make_unique<dryve_d1_bridge::D1>(dryve_d1_bridge::DRYVE_D1_IP_ADDRESS_Y_AXIS,
                                            dryve_d1_bridge::MODBUS_TCP_PORT,
                                            std::make_unique<dryve_d1_bridge::SocketWrapper>());
  std::unique_ptr<dryve_d1_bridge::D1> d1_y_axis =
      std::make_unique<dryve_d1_bridge::D1>(dryve_d1_bridge::DRYVE_D1_IP_ADDRESS_X_AXIS,
                                            dryve_d1_bridge::MODBUS_TCP_PORT,
                                            std::make_unique<dryve_d1_bridge::SocketWrapper>());
  std::unique_ptr<dryve_d1_bridge::D1> d1_rotating_arm =
      std::make_unique<dryve_d1_bridge::D1>(dryve_d1_bridge::DRYVE_D1_IP_ADDRESS_ROTATING_ARM,
                                            dryve_d1_bridge::MODBUS_TCP_PORT,
                                            std::make_unique<dryve_d1_bridge::SocketWrapper>());

  d1_x_axis->close_connection();
  d1_y_axis->close_connection();
  d1_rotating_arm->close_connection();

  return 0;
}
