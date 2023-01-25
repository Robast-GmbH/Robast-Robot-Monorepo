#ifndef RB_THERON__DRAWER_SIMULATION_HPP_
#define RB_THERON__DRAWER_SIMULATION_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "communication_interfaces/msg/drawer.hpp"
#include "communication_interfaces/msg/drawer_leds.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"

namespace drawer_simulation
{

    class DrawerSimulation : public rclcpp::Node
    {
        public:
            using DrawerAddress = communication_interfaces::msg::DrawerAddress;
            using DrawerLeds = communication_interfaces::msg::DrawerLeds;
            using DrawerStatus = communication_interfaces::msg::DrawerStatus;

            DrawerSimulation();
            ~DrawerSimulation() {};

        private:
            rclcpp::Subscription<DrawerAddress>::SharedPtr open_drawer_subscription_;
            rclcpp::Subscription<DrawerLeds>::SharedPtr drawer_leds_subscription_;
            rclcpp::Publisher<DrawerStatus>::SharedPtr drawer_status_publisher_;

            std::shared_ptr<rclcpp::Node> get_shared_pointer_of_node();

            void open_drawer_topic_callback(const DrawerAddress& msg);

            void drawer_leds_topic_callback(const DrawerLeds& msg);

            void send_drawer_is_open_feedback(communication_interfaces::msg::DrawerStatus drawer_status_msg, uint8_t drawer_id);

    };
}
#endif //RB_THERON__DRAWER_SIMULATION_HPP_