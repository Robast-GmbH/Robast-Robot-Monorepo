#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>
#include <rclcpp/qos.hpp>


#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class DrawerOpenAction : public plansys2::ActionExecutorClient
{
public:
    DrawerOpenAction()
    : plansys2::ActionExecutorClient("drawer_open", 500ms)
    {
    auto qos_ = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);
    
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
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_vel_pub_->on_activate();


    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  
  void do_work()
  {
  }

  rclcpp::Subscription<communication_interface::msg::DrawerStatus>::SharedPtr drawer_status_sub_;
  communication_interface::msg::DrawerStatus drawer_status_;

  double dist_to_move;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DrawerOpenAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "drawer_open"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
