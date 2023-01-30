#ifndef DRAWER_NFC_TREE_INITIATOR__BT_BASE_NODES_HPP
#define DRAWER_NFC_TREE_INITIATOR__BT_BASE_NODES_HPP

#include <functional>
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>


#include "bt_base_nodes/bt_sub_base.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"

namespace bt_base_nodes
{
    class DrawerNFCTreeInitiator: public bt_base_nodes::BTSubBase<communication_interfaces::msg::DrawerAddress> {

    public:
        DrawerNFCTreeInitiator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): BTSubBase(options)
        {}

    protected:
        void callbackRunBT(const std_msgs::msg::String::SharedPtr msg)
        {
            _blackboard->set<std::string>("user_access_name", *msg);
            _bt.tickWhileRunning(std::chrono::milliseconds(10));
        }
    };
}

#endif