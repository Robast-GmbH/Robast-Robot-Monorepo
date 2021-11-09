#ifndef REMOTE_PLUGIN_HPP_
#define REMOTE_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class RemotePluginPrivate;



/**
  Example Usage:
  \code{.xml}
    <plugin name="RemotePlugin" filename="libremotePlugin.so">
      <ros>
        <!-- Add a namespace -->
        <namespace>/demo</namespace>
        <!-- Remap the default topic -->
        <remapping>cmd_vel:=custom_cmd_vel</remapping>
        <remapping>odom:=custom_odom</remapping>
      </ros>
      <update_rate>100</update_rate>
      <publish_rate>10</publish_rate>
      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_frame>odom_demo</odometry_frame>
      <robot_base_frame>link</robot_base_frame>
      <covariance_x>0.0001</covariance_x>
      <covariance_y>0.0001</covariance_y>
      <covariance_yaw>0.01</covariance_yaw>
    </plugin>
  \endcode
*/

class RemotePlugin : public gazebo::ModelPlugin
{
public:
  /// Constructor
  RemotePlugin();

  /// Destructor
  ~RemotePlugin();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<RemotePluginPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // REMOTE_PLUGIN_HPP_