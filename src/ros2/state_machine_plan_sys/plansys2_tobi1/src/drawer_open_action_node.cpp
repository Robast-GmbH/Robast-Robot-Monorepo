#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>
#include <rclcpp/qos.hpp>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class DrawerOpenAction : public plansys2::ActionExecutorClient
{
public:
  DrawerOpenAction()
      : plansys2::ActionExecutorClient("drawer_open", 500ms)
  { 
    qos_ = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    sent_ = false;

    using namespace std::placeholders;
    drawer_status_sub_ = create_subscription<communication_interface::msg::DrawerStatus>(
        "/drawer_is_open",
        qos_,
        std::bind(&DrawerOpenAction::current_pos_callback, this, _1));
  }

  void current_pos_callback(const communication_interface::msg::DrawerStatus::SharedPtr msg)
  {
    drawer_status_ = msg;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    open_drawer_pub_ = this->create_publisher<communication_interface::msg::DrawerAddress>("/open_drawer", qos_);
    led_pub_ = this->create_publisher<communication_interface::msg::DrawerLeds>("/drawer_leds", qos_);
    open_drawer_pub_->on_activate();
    led_pub_->on_activate();
    sent_ = false;

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    open_drawer_pub_->on_deactivate();
    led_pub_->on_deactivate();
    sent_ = false;

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void do_work()
  {
    if (!sent_)
    {
      /* messages senden auf die publisher */
    }
  }

  rclcpp::Subscription<communication_interface::msg::DrawerStatus>::SharedPtr drawer_status_sub_;
  communication_interface::msg::DrawerStatus drawer_status_;
  rclcpp::QoS qos_;

  bool sent_
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DrawerOpenAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "drawer_open"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
