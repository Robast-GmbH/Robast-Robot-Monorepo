#ifndef DRAWER_TREE_INITIATOR__BT_BASE_NODES_HPP
#define DRAWER_TREE_INITIATOR__BT_BASE_NODES_HPP

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
    class DrawerTreeInitiator: public bt_base_nodes::BTSubBase<communication_interfaces::msg::DrawerAddress> {

    public:
        DrawerTreeInitiator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): BTSubBase(options)
        {}

    protected:
        void callbackRunBT(const communication_interfaces::msg::DrawerAddress::SharedPtr msg)
        {
            _blackboard->set<communication_interfaces::msg::DrawerAddress>("drawer_address", *msg);
            _bt.tickWhileRunning(std::chrono::milliseconds(10));
        }
    };
}
#endif

