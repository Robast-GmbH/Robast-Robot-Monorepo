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
    class DrawerTreeInitiator : public bt_base_nodes::BTSubBase<communication_interfaces::msg::DrawerAddress> {

    public:
        DrawerTreeInitiator(std::string bt_path, std::vector<std::string> plugins) : BTSubBase(bt_path, plugins)
        {
            communication_interfaces::msg::DrawerAddress tmp;
            tmp.drawer_controller_id = 0;
            tmp.drawer_id = 10;
            _blackboard->set<communication_interfaces::msg::DrawerAddress>("drawer_address", tmp);            
            _bt = bt_engine_->createTreeFromFile(bt_path, _blackboard);
            init_subscriber("open_request");
        }

    protected:
        void callbackRunBT(const communication_interfaces::msg::DrawerAddress::SharedPtr msg)
        {
            _blackboard->set<communication_interfaces::msg::DrawerAddress>("drawer_address", *msg);
            _bt.tickWhileRunning(std::chrono::milliseconds(10));
        }
    };
}
#endif

int main(int argc, char* argv[ ])
{
    rclcpp::init(argc, argv);
    std::string path = "/workspace/install/drawer_sm/trees/trees/drawer_sequence.xml";
    const std::vector<std::string> plugins = {
        "drawer_open_request_action_bt_node",
        "change_led_action_bt_node",
        "open_drawer_action_bt_node",
        "drawer_status_condition_bt_node"
    };
    rclcpp::spin(std::make_shared<bt_base_nodes::DrawerTreeInitiator>(path, plugins));
    rclcpp::shutdown();
    return 0;
}