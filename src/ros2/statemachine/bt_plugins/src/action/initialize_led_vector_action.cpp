#include "bt_plugins/action/initialize_led_vector_action.hpp"
// TODO @Tobi maybe add https://stackoverflow.com/questions/25108854/initializing-the-size-of-a-c-vector to some conflence guide page

namespace bt_plugins
{
  InitLEDVector::InitLEDVector(
      const std::string &name,
      const BT::NodeConfig &config) : BT::SyncActionNode(name, config)
  {
    int a = 0;
    getInput("size", a);
    _size = (uint8_t)a;
    _led_vector = std::vector<bt_base_types::LED>(_size, bt_base_types::LED{0, 0, 0, 0});
  }

  BT::NodeStatus InitLEDVector::tick()
  {
    setOutput("led_vector", _led_vector);
    return BT::NodeStatus::SUCCESS;
  }
}
// namespace bt_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_plugins::InitLEDVector>("InitLEDVector");
}