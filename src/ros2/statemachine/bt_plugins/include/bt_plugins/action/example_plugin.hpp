#include "behaviortree_cpp/action_node.h"

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace drawer_statemachine
{

  class Hans: public BT::SyncActionNode
  {
  public:
    Hans(const std::string& name,
        const BT::NodeConfig& config); 
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
        {
            return
            {                
            };
        }
  };

}