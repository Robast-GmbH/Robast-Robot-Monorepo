#include "bt_plugins/action/example_plugin.hpp"

namespace drawer_statemachine
{
    Hans::Hans(const std::string& name,
        const BT::NodeConfig& config):
      BT::SyncActionNode(name, config)
    {}

    // You must override the virtual function tick()
    BT::NodeStatus Hans::tick() 
    {
      std::cout << "test: " << this->name() << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::Hans>("Hans");
}