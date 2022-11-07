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
#include "led_colors.hpp"

#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>

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
    on_activate(const rclcpp_lifecycle::State& previous_state)
  {
    open_drawer_pub_ = this->create_publisher<communication_interface::msg::DrawerAddress>("/open_drawer", qos_);
    led_pub_ = this->create_publisher<communication_interface::msg::DrawerLeds>("/drawer_leds", qos_);
    open_drawer_pub_->on_activate();
    led_pub_->on_activate();
    sent_ = false;

    std::string drawer = get_arguments()[1];
    std::string led_color = get_arguments()[3];

    const std::string delimiter = "_";
    std::string drawer_id = drawer.substr(0, drawer.find(delimiter));
    drawer.erase(0, drawer.find(delimiter) + delimiter.length());
    std::string drawer_controller_id = drawer;

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state)
  {
    open_drawer_pub_->on_deactivate();
    led_pub_->on_deactivate();
    sent_ = false;

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void import_settings()
  {
    std::ifstream file("/workspace/src/state_machine_plan_sys/plansys2_tobi1/config/config.json");
    Json::Reader reader;
    Json::Value configJsonData;
    reader.parse(file, configJsonData);
    for (int i = 0; i < configJsonData["colors"].size(); i++)
    {
      led_color color = { configJsonData["colors"][i]["red"].asUInt(),
      configJsonData["colors"][i]["blue"].asUInt(),
      configJsonData["colors"][i]["green"].asUInt(),
      configJsonData["colors"][i]["brightness"].asUInt(),
      configJsonData["colors"][i]["mode"].asUInt() };
      available_colors_.insert({ configJsonData["colors"][i]["id"], color });
    }
  }

  void do_work()
  {
    if (!sent_)
    {

      /* messages senden auf die publisher */
    }
  }

  rclcpp::Subscription<communication_interface::msg::DrawerStatus>::SharedPtr drawer_status_sub_;
  rclcpp_lifecycle::LifecyclePublisher<communication_interface::msg::DrawerAddress>::SharedPtr open_drawer_pub_;
  rclcpp_lifecycle::LifecyclePublisher<communication_interface::msg::DrawerLeds>::SharedPtr led_pub_;
  communication_interface::msg::DrawerStatus drawer_status_;
  rclcpp::QoS qos_;

  bool sent_;
  std::map<std::string, led_color> available_colors_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DrawerOpenAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "drawer_open"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
