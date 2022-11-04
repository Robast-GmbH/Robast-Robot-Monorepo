#ifndef ROBAST_MAP_UPDATE_MODULE__MAP_COMBINE_HPP_
#define ROBAST_MAP_UPDATE_MODULE__MAP_COMBINE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <forward_list>
#include <mutex>
#include <unordered_map>

#include <combine_grids/merging_pipeline.h>
#include <geometry_msgs/msg/transform.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/thread.hpp>

namespace robast_map_update
{
struct MapSubscription {
  // protects consistency of writable_map and readonly_map
  // also protects reads and writes of shared_ptrs
  std::mutex mutex;



  geometry_msgs::msg::Transform initial_pose;
  nav_msgs::msg::OccupancyGrid::SharedPtr writable_map;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr readonly_map;

  // ros::Subscriber map_sub;
  // ros::Subscriber map_updates_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_updates_sub;
};

class MapCombine : public rclcpp::Node
{
public:

        MapCombine();

private:
        double _merging_rate;
        
        // map subscriber
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _slam_map_getter;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _base_map_getter;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _merged_map_publisher;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _debug_base_map_publisher;

        combine_grids::MergingPipeline _pipeline;
        std::mutex _pipeline_mutex;

        nav_msgs::msg::OccupancyGrid::SharedPtr _slam_map;
        nav_msgs::msg::OccupancyGrid::SharedPtr _base_map;
        nav_msgs::msg::OccupancyGrid::SharedPtr _combined_map;
        rclcpp::TimerBase::SharedPtr _map_merging_timer;


        void slam_map_subscriber(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void base_map_subscriber(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);


        void updateSlamMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
        void mapCombination();


};
}  // namespace nav2
#endif  // ROBAST_MAP_UPDATE_MODULE__MAP_COMBINE_HPP_