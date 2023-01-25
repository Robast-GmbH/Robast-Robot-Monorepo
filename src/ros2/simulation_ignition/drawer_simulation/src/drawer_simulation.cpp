#include "drawer_simulation/drawer_simulation.hpp"

namespace drawer_simulation
{
    DrawerSimulation::DrawerSimulation() : Node("drawer_simulation")
    {
        RCLCPP_INFO(this->get_logger(), "Creating Drawer Simulation Node!"); // Debugging

        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        qos.avoid_ros_namespace_conventions(false);

        this->open_drawer_subscription_ = this->create_subscription<DrawerAddress>(
            "open_drawer", qos, std::bind(&DrawerSimulation::open_drawer_topic_callback, this, std::placeholders::_1));

        this->drawer_leds_subscription_ = this->create_subscription<DrawerLeds>(
            "drawer_leds", qos, std::bind(&DrawerSimulation::drawer_leds_topic_callback, this, std::placeholders::_1));

        this->drawer_status_publisher_ = this->create_publisher<DrawerStatus>("drawer_is_open", qos);
    }

    void DrawerSimulation::send_drawer_is_open_feedback(communication_interfaces::msg::DrawerStatus drawer_status_msg, uint8_t drawer_id)
    {
        RCLCPP_INFO(this->get_logger(), "Sending send_drawer_is_open_feedback with drawer_controller_id: '%i'", drawer_status_msg.drawer_address.drawer_controller_id); // Debugging
        drawer_status_msg.drawer_address.drawer_id = drawer_id;
        drawer_status_msg.drawer_is_open = true;
        this->drawer_status_publisher_->publish(drawer_status_msg);
    }

    std::shared_ptr<rclcpp::Node> DrawerSimulation::get_shared_pointer_of_node()
    {
        return std::enable_shared_from_this<rclcpp::Node>::shared_from_this();
    }

    void DrawerSimulation::drawer_leds_topic_callback(const DrawerLeds& msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard from drawer_leds topic the led mode: '%i'", msg.mode); // Debugging
    }

    void DrawerSimulation::open_drawer_topic_callback(const DrawerAddress& msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard from open_drawer topic the drawer_controller_id: '%i'", msg.drawer_controller_id); // Debugging

        uint32_t drawer_controller_id = msg.drawer_controller_id;
        uint8_t drawer_id = msg.drawer_id;

        if (drawer_controller_id == 0 && drawer_id == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid drawer_controller_id or");
            return;
        }

        auto move_group_interface = moveit::planning_interface::MoveGroupInterface(this->get_shared_pointer_of_node(), "drawer_planning_group");

        std::string drawer_joint = "drawer_" + std::to_string(drawer_controller_id) + "_joint";

        move_group_interface.setJointValueTarget(drawer_joint, 0.34);

        // Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();

        // Execute the plan
        if (success)
        {
            move_group_interface.execute(plan);
            //TODO: Report that drawer was opened successfull
            // this->send_drawer_is_open_feedback();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planing failed!");
        }
    }
}