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

/*
this->declare_parameter("joint_names", default_joint_names);
this->declare_parameter("ign_joint_topics", default_ign_joint_topics);
this->declare_parameter("rate", 200);
joint_names = this->get_parameter("joint_names").get_parameter_value().get<std::vector<std::string>>();
ign_joint_topics = this->get_parameter("ign_joint_topics").get_parameter_value().get<std::vector<std::string>>();
update_rate = this->get_parameter("rate").as_int();
*/


namespace bt_base_nodes
{
  template<class TopicT>
  class BTSubBase: public rclcpp::Node
  {
  public:
    BTSubBase(const rclcpp::NodeOptions & options):Node("bt_tickers", options)
    {   
    }

    void configure()
    {
      
      static BT::NodeConfig* config_;
      config_ = new BT::NodeConfig();
      std::string path = "/workspace/install/drawer_sm/trees/trees/drawer_sequence.xml";
      const std::vector<std::string> default_plugins = {
        "drawer_open_request_action_bt_node",
        "change_led_action_bt_node",
        "open_drawer_action_bt_node",
        "drawer_status_condition_bt_node"
      };
      this->declare_parameter("plugins", default_plugins);
      this->declare_parameter("bt_path", path);
      plugins_ = this->get_parameter("plugins").get_parameter_value().get<std::vector<std::string>>();
      bt_path_ = this->get_parameter("bt_path").as_string();
      // Create the blackboard that will be shared by all of the nodes in the tree
      _blackboard = BT::Blackboard::create();
      // Put items on the blackboard
      _blackboard->set<rclcpp::Node::SharedPtr>(
        "node",
        std::shared_ptr<rclcpp::Node>(shared_from_this()));
      _blackboard->set<std::chrono::milliseconds>(
        "bt_loop_duration",
        std::chrono::milliseconds(10));
      _bt_engine = std::make_unique<drawer_statemachine::BehaviorTreeEngine>(plugins_);
      _bt = _bt_engine->createTreeFromFile(bt_path_, _blackboard);
      init_subscriber("open_request");
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

    void init_subscriber(std::string topic = "start_bt")
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
    
  private:
    
    std::string bt_path_;
    std::vector<std::string> plugins_;
    typename rclcpp::Subscription<TopicT>::SharedPtr start_bt_sub_;

  };
}

#endif