#pragma once
#include <smacc2/client_bases/smacc_subscriber_client.hpp>

#include "communication_interfaces/msg/drawer_address.hpp"

namespace sm_drawer_gate
{
namespace cl_drawer_control
{
class ClDrawerControl : public smacc2::client_bases::SmaccSubscriberClient<communication_interfaces::msg::DrawerAddress>
{
public:
  ClDrawerControl() {}
};
}  // namespace cl_drawer_control
}  // namespace sm_drawer_gate
