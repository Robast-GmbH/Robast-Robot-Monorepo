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
          "open_drawer_action_bt_node",
          "drawer_status_condition_bt_node",
          "drawer_change_state_request_action_bt_node",
          "nfc_to_drawer_action_bt_node",
          "electric_drawer_status_condition_bt_node",
          "move_electric_drawer_action_bt_node",
          "base_error_decorator_node",
          "partial_colorize_leds_action_bt_node",
          "publish_led_action_bt_node",
          "reset_decorator_bt_node",
          "initialize_led_vector_action_bt_node",
          "bool_topic_condition_node",
          "get_blackboard_value_action_bt_node",
          "led_changed_condition_node"};
      std::string path = "/workspace/src/statemachine/drawer_sm/trees/robo_base.xml";
      this->declare_parameter("plugins", plugins_);
      this->declare_parameter("bt_path", path);
      this->declare_parameter("trigger_topic", "start_bt");
      this->declare_parameter("main_tree", "BehaviorTree");
      this->declare_parameter("tree_tick_time", 100);
      plugins_ = this->get_parameter("plugins").get_parameter_value().get<std::vector<std::string>>();
      _main_tree_name = this->get_parameter("main_tree").as_string();
      _bt_path = this->get_parameter("bt_path").as_string();
      _trigger_topic = this->get_parameter("trigger_topic").as_string();
      tree_tick_time_ = this->get_parameter("tree_tick_time").as_int();
    }

    void configure()
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
          std::chrono::milliseconds(tree_tick_time_));
      bt_engine_ = std::make_unique<statemachine::BehaviorTreeEngine>(plugins_);
      bt_ = bt_engine_->createTreeFromFile(_bt_path, blackboard_, _main_tree_name);
      init_subscriber(_trigger_topic);
    }

  protected:
    virtual void callbackRunBT(const typename TopicT::SharedPtr msg)
    {
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
    std::unique_ptr<statemachine::BehaviorTreeEngine> bt_engine_;
    std::vector<std::string> plugins_;
    uint16_t tree_tick_time_;

  private:
    std::string _bt_path;
    std::string _trigger_topic;
    std::string _main_tree_name;
    typename rclcpp::Subscription<TopicT>::SharedPtr _start_bt_sub;
  };
}

#endif