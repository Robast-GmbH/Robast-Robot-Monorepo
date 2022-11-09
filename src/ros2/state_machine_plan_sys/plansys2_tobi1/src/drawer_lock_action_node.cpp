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
#include "plansys2_tobi1/led_colors.hpp"

#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>

using namespace std::chrono_literals;

class DrawerCloseAction : public plansys2::ActionExecutorClient
{
public:
  DrawerCloseAction()
    : plansys2::ActionExecutorClient("drawer_lock", 500ms), qos_(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2))
  {
    qos_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_.avoid_ros_namespace_conventions(false);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& previous_state)
  {
    import_settings();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state)
  {
    led_pub_ = this->create_publisher<communication_interfaces::msg::DrawerLeds>("/drawer_leds", qos_);
    led_pub_->on_activate();

    std::string drawer = get_arguments()[1];
    std::string action_color = get_arguments()[4];

    communication_interfaces::msg::DrawerAddress drawer_unlock_msg;
    drawer_unlock_msg.set__drawer_id(std::get<0>(module_names_[drawer]));
    drawer_unlock_msg.set__drawer_controller_id(std::get<1>(module_names_[drawer]));

    communication_interfaces::msg::DrawerLeds drawer_led_msg = led_color::add_leds_to_msg(available_colors_[action_color]);
    drawer_led_msg.set__drawer_address(drawer_unlock_msg);
    led_pub_->publish(drawer_led_msg);

    finish(true, 1.0, "Drawer light changed to default");

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state)
  {
    led_pub_->on_deactivate();

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void import_settings()
  {
    //TODO remove hardcoded path
    std::ifstream file("/workspace/src/state_machine_plan_sys/plansys2_tobi1/config/config.json");
    Json::Reader reader;
    Json::Value configJsonData;
    reader.parse(file, configJsonData);
    for (int i = 0; i < configJsonData["colors"].size(); i++)
    {
      auto jsonColor = configJsonData["colors"];
      led_color::led_color color = { jsonColor[i]["red"].asUInt(),
      jsonColor["blue"].asUInt(),
      jsonColor["green"].asUInt(),
      jsonColor["brightness"].asUInt(),
      jsonColor["mode"].asUInt() };
      std::string color_name = configJsonData["colors"][i]["id"].asCString();
      available_colors_.insert({ color_name, color });
    }

    for (int i = 0; i < configJsonData["drawers"].size(); i++)
    {
      auto drawerJson = configJsonData["drawers"][i];
      module_names_.insert({
        drawerJson["id"].asCString(),
        {drawerJson["drawer_id"].asUInt(), drawerJson["drawer_controller_id"].asUInt()}
        });
    }
  }

  void do_work()
  {}

  rclcpp::Subscription<communication_interfaces::msg::DrawerStatus>::SharedPtr drawer_status_sub_;
  rclcpp_lifecycle::LifecyclePublisher<communication_interfaces::msg::DrawerLeds>::SharedPtr led_pub_;
  rclcpp::QoS qos_;

  bool received_msg_;
  std::map<std::string, led_color::led_color> available_colors_;
  std::map<std::string, std::tuple<uint8_t, uint32_t>> module_names_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DrawerCloseAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "drawer_lock"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
