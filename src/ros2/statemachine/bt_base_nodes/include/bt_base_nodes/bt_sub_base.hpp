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
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
#include "bt_plugins/behavior_tree_engine.hpp"


namespace bt_base_nodes
{
  template<class TopicT>
  class BTSubBase: public rclcpp::Node
  {
  public:
    BTSubBase(const rclcpp::NodeOptions& options): Node("bt_tickers", options)
    {
      _plugins = {
        "change_led_action_bt_node",
        "open_drawer_action_bt_node",
        "drawer_status_condition_bt_node",
        "nfc_to_drawer_action_bt_node"
      };
      std::string path = "/workspace/install/drawer_sm/trees/trees/drawer_sequence.xml";      
      this->declare_parameter("plugins", _plugins);
      this->declare_parameter("bt_path", path);
      _plugins = this->get_parameter("plugins").get_parameter_value().get<std::vector<std::string>>();
      bt_path_ = this->get_parameter("bt_path").as_string();

    }

    void configure(std::string trigger_topic = "start_bt")
    {
      
      static BT::NodeConfig* config_;
      config_ = new BT::NodeConfig();
      // Create the blackboard that will be shared by all of the nodes in the tree
      _blackboard = BT::Blackboard::create();
      // Put items on the blackboard
      _blackboard->set<rclcpp::Node::SharedPtr>(
        "node",
        shared_from_this());
      _blackboard->set<std::chrono::milliseconds>(
        "bt_loop_duration",
        std::chrono::milliseconds(10));
      _bt_engine = std::make_unique<drawer_statemachine::BehaviorTreeEngine>(_plugins);
      _bt = _bt_engine->createTreeFromFile(bt_path_, _blackboard);
      init_subscriber(trigger_topic);
    }

  protected:
    
    virtual void callbackRunBT(const typename TopicT::SharedPtr msg)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("BTSubBase"), "Tree starts");
      _bt.tickWhileRunning();
      RCLCPP_DEBUG(rclcpp::get_logger("BTSubBase"), "Tree ends");
    }

    void reset_subscriber()
    {
      start_bt_sub_.reset();      
    }

    virtual void init_subscriber(std::string topic = "start_bt")
    {
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.transient_local().reliable();
      start_bt_sub_ = this->create_subscription<TopicT>(
          topic,
          qos,
          std::bind(&BTSubBase::callbackRunBT, this, std::placeholders::_1)); 
    }
    
    BT::Tree _bt;
    BT::Blackboard::Ptr _blackboard;
    std::unique_ptr<drawer_statemachine::BehaviorTreeEngine> _bt_engine;
    std::vector<std::string> _plugins;
    
  private:
    
    std::string bt_path_;
    typename rclcpp::Subscription<TopicT>::SharedPtr start_bt_sub_;

  };
}

#endif