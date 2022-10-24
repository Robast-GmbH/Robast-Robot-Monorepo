#include <smacc2/smacc.hpp>

namespace sm_drawer_gate
{
namespace cl_drawer_control
{
using namespace std::chrono_literals;

class CpDrawerAddress : public smacc2::ISmaccComponent
{
public:
  communication_interfaces::msg::DrawerAddress drawer_address_;

  void onInitialize() override
  {
    auto client_ =
      dynamic_cast<smacc2::client_bases::SmaccSubscriberClient<communication_interfaces::msg::DrawerAddress> *>(
        owner_);
    client_->onMessageReceived(&CpDrawerAddress::MessageCallbackDrawerAddress, this);
  }

  void MessageCallbackDrawerAddress(const communication_interfaces::msg::DrawerAddress & msg)
  {
    this->drawer_address_ = msg;
    RCLCPP_INFO(getLogger(), " +++++++++++++++++++++++++++ MessageCallbackDrawerAddress in CpDrawerAddress! %i", 1);
  }

  communication_interfaces::msg::DrawerAddress getLastMsg()
  {
    return this->drawer_address_;
  }
};
}  // namespace cl_drawer_control
}  // namespace sm_drawer_gate
