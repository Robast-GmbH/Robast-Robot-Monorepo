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
    BTSubBase(std::string bt_path, std::vector<std::string> plugins):Node("bt_tickers"), plugins_(plugins)
    {
      static BT::NodeConfig* config_;
      config_ = new BT::NodeConfig();
      // Create the blackboard that will be shared by all of the nodes in the tree
      _blackboard = BT::Blackboard::create();
      // Put items on the blackboard
      _blackboard->set<rclcpp::Node::SharedPtr>(
        "node",
        std::shared_ptr<rclcpp::Node>(this));
      _blackboard->set<std::chrono::milliseconds>(
        "bt_loop_duration",
        std::chrono::milliseconds(10));
      bt_engine_ = std::make_unique<drawer_statemachine::BehaviorTreeEngine>(plugins_);
      // bt_engine_ = std::make_unique<drawer_statemachine::BehaviorTreeEngine>(plugins_);
      // _bt = bt_engine_->createTreeFromFile(bt_path, _blackboard);
      // init_subscriber("default");
    }

  protected:
    
    virtual void callbackRunBT(const typename TopicT::SharedPtr msg)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("BTSubBase"), "Baum Startet");
      _bt.tickWhileRunning();
      RCLCPP_DEBUG(rclcpp::get_logger("BTSubBase"), "Baum Vorbei");
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
          std::bind(&BTSubBase::callbackRunBT, this, std::placeholders::_1)   ); 
    }
    
    BT::Tree _bt;
    BT::Blackboard::Ptr _blackboard;
    std::unique_ptr<drawer_statemachine::BehaviorTreeEngine> bt_engine_;
    
  private:
    
    const std::vector<std::string> plugins_;
    typename rclcpp::Subscription<TopicT>::SharedPtr start_bt_sub_;

  };
}

#endif