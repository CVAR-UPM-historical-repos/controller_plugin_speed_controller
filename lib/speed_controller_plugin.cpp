/*!*******************************************************************************************
 *  \file       speed_controller_plugin.cpp
 *  \brief      Speed PID controller plugin for the Aerostack framework.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "speed_controller_plugin.hpp"

namespace controller_plugin_speed_controller
{

  void SCPlugin::ownInitialize()
  {
    flags_.parameters_read = false;
    flags_.state_received = false;
    flags_.ref_received = false;

    controller_handler_ = std::make_shared<SpeedController>();

    static auto parameters_callback_handle_ = node_ptr_->add_on_set_parameters_callback(
      std::bind(&SCPlugin::parametersCallback, this, std::placeholders::_1));

    declareParameters();
    
    resetState();
    resetReferences();
    resetCommands();
    return;
  };

  void SCPlugin::updateState(const nav_msgs::msg::Odometry &odom_msg)
  {
    uav_state_.pos = Vector3d(
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z);

    uav_state_.vel = Vector3d(
        odom_msg.twist.twist.linear.x,
        odom_msg.twist.twist.linear.y,
        odom_msg.twist.twist.linear.z);

    tf2::Quaternion q_tf(
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w);

    uav_state_.rot = q_tf;

    flags_.state_received = true;
    return;
  };

  void SCPlugin::updateReference(const geometry_msgs::msg::PoseStamped &pose_msg)
  {
    if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED ||
        control_mode_in_.yaw_mode != as2_msgs::msg::ControlMode::YAW_ANGLE)
    {
      return;
    }

    Eigen::Quaterniond q(
        pose_msg.pose.orientation.w,
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z);

    control_ref_.yaw[0] = q.toRotationMatrix().eulerAngles(0, 1, 2)[2];

    flags_.ref_received = true;
    return;
  };

  void SCPlugin::updateReference(const geometry_msgs::msg::TwistStamped &twist_msg)
  {
    if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED)
    {
      return;
    }

    control_ref_.vel = Vector3d(
        twist_msg.twist.linear.x,
        twist_msg.twist.linear.y,
        twist_msg.twist.linear.z);

    if (control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED)
    {
      control_ref_.yaw[1] = twist_msg.twist.angular.z;
    }

    flags_.ref_received = true;
    return;
  };

  void SCPlugin::updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &traj_msg)
  {
    if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::TRAJECTORY)
    {
      return;
    }

    control_ref_.pos = Vector3d(
        traj_msg.positions[0],
        traj_msg.positions[1],
        traj_msg.positions[2]);

    control_ref_.vel = Vector3d(
        traj_msg.velocities[0],
        traj_msg.velocities[1],
        traj_msg.velocities[2]);

    control_ref_.yaw = Vector3d(
        traj_msg.positions[4],
        traj_msg.velocities[4],
        traj_msg.accelerations[4]);

    flags_.ref_received = true;
    return;
  };

  bool SCPlugin::setMode(const as2_msgs::msg::ControlMode &in_mode,
                         const as2_msgs::msg::ControlMode &out_mode)
  {
    control_mode_in_ = in_mode;
    control_mode_out_ = out_mode;

    flags_.ref_received = false;
    flags_.state_received = false;

    last_time_ = node_ptr_->now();

    return true;
  };

  void SCPlugin::computeOutput(geometry_msgs::msg::PoseStamped &pose,
                               geometry_msgs::msg::TwistStamped &twist,
                               as2_msgs::msg::Thrust &thrust)
  {
    if (!flags_.state_received)
    {
      auto& clk = *node_ptr_->get_clock();
      RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "State not received yet");
      return;
    }

    if (!flags_.parameters_read)
    {
      auto& clk = *node_ptr_->get_clock();
      RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Parameters not read yet");
      return;
    }

    if (!flags_.ref_received)
    {
      auto& clk = *node_ptr_->get_clock();
      RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "State changed, but ref not recived yet");
      return;
    }
    else
    {
      computeActions(pose, twist, thrust);
    }

    static rclcpp::Time last_time_ = node_ptr_->now();
    return;
  };

  void SCPlugin::computeActions(geometry_msgs::msg::PoseStamped &pose,
                                geometry_msgs::msg::TwistStamped &twist,
                                as2_msgs::msg::Thrust &thrust)
  {
    resetCommands();

    rclcpp::Time current_time = node_ptr_->now();
    double dt = (current_time - last_time_).nanoseconds() / 1.0e9;
    last_time_ = current_time;

    switch (control_mode_in_.control_mode)
    {
    case as2_msgs::msg::ControlMode::HOVER:
    {
      return;
      break;
    } 
    case as2_msgs::msg::ControlMode::POSITION:
    {
      control_command_.vel = controller_handler_->computePositionControl(
          uav_state_,
          control_ref_,
          dt);
      break;
    } 
    case as2_msgs::msg::ControlMode::SPEED:
    {
      control_command_.vel = control_ref_.vel;
      break;
    } 
    case as2_msgs::msg::ControlMode::TRAJECTORY:
    {
      control_command_.vel = controller_handler_->computeTrayectoryControl(
          uav_state_,
          control_ref_,
          dt);
      break;
    } 
    default:
      auto& clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown control mode");
      return;
      break;
    }

    switch (control_mode_in_.yaw_mode)
    {
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
    {    
      tf2::Matrix3x3 m(uav_state_.rot);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      
      control_command_.yaw[1] = controller_handler_->computeYawSpeed(
          yaw,
          (double) control_ref_.yaw[0],
          dt);

      break;
    }
    case as2_msgs::msg::ControlMode::YAW_SPEED:
    {  
      control_command_.yaw[1] = control_ref_.yaw[1];
      break;
    }
    default:
      auto& clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown yaw mode");
      return;
      break;
    }

    switch (control_mode_in_.reference_frame)
    {
    case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
    {
      getOutput(pose, twist, thrust);
      break;
    }
    default:
      auto& clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown reference frame");
      return;
      break;
    }

    return;
  };

  void SCPlugin::getOutput(geometry_msgs::msg::PoseStamped &pose_msg,
                           geometry_msgs::msg::TwistStamped &twist_msg,
                           as2_msgs::msg::Thrust &thrust_msg)
  {
    twist_msg.header.stamp = node_ptr_->now();

    twist_msg.twist.linear.x = control_command_.vel[0];
    twist_msg.twist.linear.y = control_command_.vel[1];
    twist_msg.twist.linear.z = control_command_.vel[2];

    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = control_command_.yaw[1];

    return;
  };

  rcl_interfaces::msg::SetParametersResult SCPlugin::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (auto &param : parameters)
    {
      if (controller_handler_->isParameter(param.get_name()))
      {
        controller_handler_->setParameter(param.get_name(), param.get_value<double>());

        // Remove the parameter from the list of parameters to be read
        parameters_to_read_.erase(
            std::remove(
                parameters_to_read_.begin(),
                parameters_to_read_.end(),
                param.get_name()),
            parameters_to_read_.end());
        if (parameters_to_read_.empty())
        {
          RCLCPP_DEBUG(node_ptr_->get_logger(), "All parameters read");
          flags_.parameters_read = true;
        }
      }
      else
      {
        RCLCPP_WARN(node_ptr_->get_logger(), "Parameter %s not defined in controller params", param.get_name().c_str());
        result.successful = false;
        result.reason = "parameter not found";
      }
    }
    return result;
  };

  void SCPlugin::declareParameters()
  {
    std::vector<std::pair<std::string, double>> params = controller_handler_->getParametersList();
    for (auto &param : params)
    {
      node_ptr_->declare_parameter(param.first);
    }
    return;
  };

    void SCPlugin::resetState()
  {
    uav_state_.pos = Vector3d::Zero();
    uav_state_.vel = Vector3d::Zero();
    uav_state_.rot = tf2::Quaternion::getIdentity();
    return;
  };

  void SCPlugin::resetReferences()
  {
    control_ref_.pos = uav_state_.pos;
    control_ref_.vel = Vector3d::Zero();

    tf2::Matrix3x3 tf_matrix(uav_state_.rot);
    double roll, pitch, yaw;
    tf_matrix.getRPY(roll, pitch, yaw);

    control_ref_.yaw = Vector3d(
        yaw,
        0.0f,
        0.0f);

    return;
  };

  void SCPlugin::resetCommands()
  {
    control_command_.vel.setZero();
    control_command_.yaw.setZero();
    return;
  };

} // namespace controller_plugin_differential_flatness

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugin_speed_controller::SCPlugin,
                       controller_plugin_base::ControllerBase)
