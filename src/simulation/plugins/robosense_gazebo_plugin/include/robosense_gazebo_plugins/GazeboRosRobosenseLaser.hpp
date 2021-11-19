
#ifndef GAZEBO_ROS_Robosense_LASER_H_
#define GAZEBO_ROS_Robosense_LASER_H_

#include <sdf/Param.hh>

#include <gazebo/transport/Node.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gazebo_ros/node.hpp>

namespace gazebo
{

  class GazeboRosRobosenseLaser : public SensorPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosRobosenseLaser();

    /// \brief Destructor
    public: ~GazeboRosRobosenseLaser();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Subscribe on-demand
    private: void ConnectCb();

    /// \brief The parent ray sensor
    private: sensors::SensorPtr parent_ray_sensor_;

    private: gazebo_ros::Node::SharedPtr ros_node_;

    /// \brief ROS publisher
    private: rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    /// \brief ROS timer to emulate publisher connection callback
    private: rclcpp::TimerBase::SharedPtr timer_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief organize cloud
    private: bool organize_cloud_;

    /// \brief the intensity beneath which points will be filtered
    private: double min_intensity_;

    /// \brief Minimum range to publish
    private: double min_range_;

    /// \brief Maximum range to publish
    private: double max_range_;

    /// \brief Gaussian noise
    private: double gaussian_noise_;

    /// \brief Gaussian noise generator
    private: static double gaussianKernel(double mu, double sigma)
    {
      // using Box-Muller transform to generate two independent standard normally distributed normal variables
      // see wikipedia
      double U = (double)rand() / (double)RAND_MAX; // normalized uniform random variable
      double V = (double)rand() / (double)RAND_MAX; // normalized uniform random variable
      return sigma * (sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V)) + mu;
    }

    /// \brief A mutex to lock access
    private: std::mutex lock_;

    // Subscribe to gazebo laserscan
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr sub_;
    private: void OnScan(const ConstLaserScanStampedPtr &_msg);

  };

} // namespace gazebo

#endif /* GAZEBO_ROS_Robosense_LASER_H_ */
