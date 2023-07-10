#ifndef BT_SUB_BASE__BT_BASE_NODES_HPP
#define BT_SUB_BASE__BT_BASE_NODES_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "bt_plugins/behavior_tree_engine.hpp"

namespace bt_base_nodes
{
  template <class TopicT>
  class BTSubBase : public rclcpp::Node
  {
  public:
    BTSubBase(const rclcpp::NodeOptions &options) : Node("bt_tickers", options)
    {
      plugins_ = {
          "change_led_action_bt_node",
          "open_drawer_action_bt_node",
          "drawer_status_condition_bt_node",
          "drawer_change_state_request_action_bt_node",
          "nfc_to_drawer_action_bt_node",
          "electric_drawer_status_condition_bt_node",
          "move_electric_drawer_action_bt_node",
          "base_error_decorator_node"};
      std::string path = "/workspace/install/drawer_sm/trees/trees/default_electrical_drawer.xml";
      this->declare_parameter("plugins", plugins_);
      this->declare_parameter("bt_path", path);
      plugins_ = this->get_parameter("plugins").get_parameter_value().get<std::vector<std::string>>();
      _bt_path = this->get_parameter("bt_path").as_string();
    }

    void configure(std::string trigger_topic = "start_bt", std::string maintree_name = "MainTree")
    {
      static BT::NodeConfig *config_;
      config_ = new BT::NodeConfig();
      // Create the blackboard that will be shared by all of the nodes in the tree
      blackboard_ = BT::Blackboard::create();
      // Put items on the blackboard
      blackboard_->set<rclcpp::Node::SharedPtr>(
          "node",
          shared_from_this());
      blackboard_->set<std::chrono::milliseconds>(
          "bt_loop_duration",
          std::chrono::milliseconds(10));
      blackboard_->set<std::chrono::steady_clock::time_point>(
          "transition_time",
          std::chrono::steady_clock::now());
      bt_engine_ = std::make_unique<drawer_statemachine::BehaviorTreeEngine>(plugins_);
      bt_ = bt_engine_->createTreeFromFile(_bt_path, blackboard_, maintree_name);
      init_subscriber(trigger_topic);
    }

  protected:
    virtual void callbackRunBT(const typename TopicT::SharedPtr msg)
    {
      blackboard_->set<std::chrono::steady_clock::time_point>(
          "transition_time",
          std::chrono::steady_clock::now());
      RCLCPP_DEBUG(rclcpp::get_logger("BTSubBase"), "Tree starts");
      bt_.tickWhileRunning();
      RCLCPP_DEBUG(rclcpp::get_logger("BTSubBase"), "Tree ends");
    }

    void reset_subscriber()
    {
      _start_bt_sub.reset();
    }

    virtual void init_subscriber(std::string topic = "start_bt")
    {
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.transient_local().reliable();
      _start_bt_sub = this->create_subscription<TopicT>(
          topic,
          qos,
          std::bind(&BTSubBase::callbackRunBT, this, std::placeholders::_1));
    }

    BT::Tree bt_;
    BT::Blackboard::Ptr blackboard_;
    std::unique_ptr<drawer_statemachine::BehaviorTreeEngine> bt_engine_;
    std::vector<std::string> plugins_;

  private:
    std::string _bt_path;
    typename rclcpp::Subscription<TopicT>::SharedPtr _start_bt_sub;
  };
}

#endif