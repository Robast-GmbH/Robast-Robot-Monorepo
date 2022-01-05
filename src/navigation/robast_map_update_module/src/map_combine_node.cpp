
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "map_combine_node.hpp"

namespace robast_map_update
{
map_combine::map_combine(): Node("map_combine_node")
{
        
        using std::placeholders::_1;

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        slam_map_getter = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/slam_map", qos, std::bind(&map_combine::slam_map_subscriber, this, _1));

        base_map_getter = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/base_map", qos, std::bind(&map_combine::base_map_subscriber, this, _1));

        // map publisher
        merged_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        // Timers
        map_merging_timer_ = this->create_wall_timer(
                std::chrono::milliseconds((uint16_t)(1000.0 / merging_rate_)),
                [this]() { mapMerging(); });
}

void map_combine::slam_map_subscriber(const nav_msg::::SharedPtr msg)
{
        combine_maps();
}

void map_combine::base_map_subscriber(const nav_msg::::SharedPtr msg)
{
        _base_map = msg;
        RCLCPP_INFO(this->get_logger(), "Received new base map");
}

}  // namespace robast_map_update


int main(int argc, char * argv[])
{
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared< robast_map_update::map_combine>());
        rclcpp::shutdown();
        return 0;
}

