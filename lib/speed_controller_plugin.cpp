#include "speed_controller_plugin.hpp"

namespace controller_plugin_speed_controller
{

  void SCPlugin::ownInitialize()
  {
      return;
  };

  void SCPlugin::updateState(const nav_msgs::msg::Odometry &odom)
  {
    return;
  };

  void SCPlugin::updateReference(const geometry_msgs::msg::PoseStamped &pose_msg)
  {
    return;
  };

  void SCPlugin::updateReference(const geometry_msgs::msg::TwistStamped &twist_msg)
  {
    return;
  };

  void SCPlugin::updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &traj_msg)
  {
    return;
  };

  void SCPlugin::computeOutput(geometry_msgs::msg::PoseStamped &pose,
                               geometry_msgs::msg::TwistStamped &twist,
                               as2_msgs::msg::Thrust &thrust)
  {
    return;
  };

  bool SCPlugin::setMode(const as2_msgs::msg::ControlMode &in_mode,
                         const as2_msgs::msg::ControlMode &out_mode)
  {
    return true;
  };

} // namespace controller_plugin_differential_flatness

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugin_speed_controller::SCPlugin,
                       controller_plugin_base::ControllerBase)
