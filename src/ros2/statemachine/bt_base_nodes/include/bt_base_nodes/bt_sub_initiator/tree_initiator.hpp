#ifndef BT_BASE_NODES__BT_SUB_INITIATOR__TREE_INITIATOR_HPP_
#define BT_BASE_NODES__BT_SUB_INITIATOR__TREE_INITIATOR_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "bt_base_nodes/bt_sub_base.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"

namespace bt_base_nodes
{
    class TreeInitiator : public bt_base_nodes::BTSubBase<communication_interfaces::msg::DrawerAddress>
    {

    public:
        TreeInitiator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : BTSubBase(options)
        {
        }

    protected:
        void callbackRunBT(const communication_interfaces::msg::DrawerAddress::SharedPtr msg)
        {
            blackboard_->set<communication_interfaces::msg::DrawerAddress>("drawer_address", *msg);
            bt_.tickWhileRunning(std::chrono::milliseconds(10));
        }
    };
}
#endif
