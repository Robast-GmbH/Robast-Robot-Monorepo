#include "initial_pose_publisher.hpp"


InitialPoseNode::InitialPoseNode(): Node("initial_pose_node"){
        _set_initial_pose_subscriber = this->create_subscription<geometry_msgs::msg::Point>(
            "/set_initial_pose",
            10,
            std::bind(&InitialPoseNode::set_initial_pose_callback, this, std::placeholders::_1)
        );

        _initial_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            10
        );
    }

void InitialPoseNode::set_initial_pose_callback(const geometry_msgs::msg::Point::SharedPtr point){
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";
        msg.header.stamp = this->get_clock()->now();
        msg.pose.pose.position.x = point->x;
        msg.pose.pose.position.y = point->y;
        msg.pose.pose.orientation.z = std::sin(point->z / 2.0);
        msg.pose.pose.orientation.w = std::cos(point->z / 2.0);

        _initial_pose_publisher->publish(msg);
    }
