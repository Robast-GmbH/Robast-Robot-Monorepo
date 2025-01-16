#ifndef DRAWER_STATEMACHINE_BOOL_TOPIC_CONDITION_HPP_
#define DRAWER_STATEMACHINE_BOOL_TOPIC_CONDITION_HPP_

#include "bt_plugins/condition/base_compare_condition.hpp"
#include "std_msgs/msg/bool.hpp"

namespace statemachine
{
  class BoolTopicCondition : public BaseCompareCondition<std_msgs::msg::Bool, bool>
  {
   public:
    BoolTopicCondition(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<bool>("target_value", "true"),
              BT::InputPort<std::string>("topic", "/robot/safety_module/safety_stop")};
    }
    BT::NodeStatus tick() override;

   protected:
    bool new_value_received_ = false;
    bool comparator(std_msgs::msg::Bool last_message, bool target_value) override;
    void callbackTopicFeedback(const std_msgs::msg::Bool::SharedPtr msg) override;
    void initialize_target_value() override;

   private:
    bool _compare_result = false;
  };
}   // namespace statemachine

#endif   // DRAWER_STATEMACHINE_BOOL_TOPIC_CONDITION_HPP_
