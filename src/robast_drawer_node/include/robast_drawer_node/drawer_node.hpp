// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROBAST_DRAWER_NODE__DRAWER_NODE_HPP_
#define ROBAST_DRAWER_NODE__DRAWER_NODE_HPP_

#include <memory>

#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robast_msgs/action/control_drawer.hpp"

namespace robast_drawer_node
{

enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};

/**
 * @class robast_drawer_node::DrawerController
 * @brief A Controller that sends commands via the CAN bus to the drawer microcontroller to open them.
 */
class DrawerController : public nav2_util::LifecycleNode
{
public:
  using ActionT = robast_msgs::action::ControlDrawer;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  /**
   * @brief A constructor for robast_drawer_node::DrawerController class
   */
  DrawerController();
  /**
   * @brief A destructor for robast_drawer_node::DrawerController class
   */
  ~DrawerController();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "DrawerController"
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Helper function to check if the request for the action is valid
   * @param goal Action goal that is checked for validity
   * @param result Action result that is used in case the request is invalid and action is canceled
   * @return True if valid, false if not valid
   */
  bool is_request_valid(const std::shared_ptr<const typename ActionT::Goal> goal,
                        std::shared_ptr<ActionT::Result> result);

  /**
   * @brief Helper function to send the succeeded action result
   * @param goal Action goal
   * @param result Action result that is filled within the action and sent to the action server
   */
  void send_succeeded_action_result(const std::shared_ptr<const typename ActionT::Goal> goal,
                        std::shared_ptr<ActionT::Result> result);


  /**
   * @brief Action server execution callback
   */
  void open_drawer();

  // Our action server
  std::unique_ptr<ActionServer> action_server_;
};

}  // namespace robast_drawer_node

#endif  // ROBAST_DRAWER_NODE__DRAWER_NODE_HPP_
