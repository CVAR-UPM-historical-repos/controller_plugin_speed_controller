/*!*******************************************************************************************
 *  \file       speed_controller_plugin.hpp
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

#ifndef __DF_PLUGIN_H__
#define __DF_PLUGIN_H__

// Std libraries
#include <array>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <unordered_map>
#include <vector>

#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "controller_plugin_base/controller_base.hpp"

#include "speed_controller.hpp"

namespace controller_plugin_speed_controller
{
  using Vector3d = Eigen::Vector3d;
  using SpeedController = speed_controller::SpeedController;
  using UAV_state = speed_controller::UAV_state;
  using Control_ref = speed_controller::Control_ref;
  using Control_command = speed_controller::Control_command;

  struct Control_flags
  {
    bool parameters_read;
    bool state_received;
    bool ref_received;
    bool hover_mode_;
  };

  class Plugin : public controller_plugin_base::ControllerBase
  {
  public:
    Plugin(){};
    ~Plugin(){};

  public:
    void ownInitialize() override;
    void updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                     const geometry_msgs::msg::TwistStamped &twist_msg) override;

    void updateReference(const geometry_msgs::msg::PoseStamped &ref) override;
    void updateReference(const geometry_msgs::msg::TwistStamped &ref) override;
    void updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &ref) override;

    bool setMode(const as2_msgs::msg::ControlMode &mode_in,
                 const as2_msgs::msg::ControlMode &mode_out) override;

    void computeOutput(geometry_msgs::msg::PoseStamped &pose,
                       geometry_msgs::msg::TwistStamped &twist,
                       as2_msgs::msg::Thrust &thrust) override;

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  
  private:
    rclcpp::Time last_time_;

    as2_msgs::msg::ControlMode control_mode_in_;
    as2_msgs::msg::ControlMode control_mode_out_;

    Control_flags flags_;

    std::shared_ptr<SpeedController> controller_handler_;

    std::vector<std::string> parameters_to_read_ = {
        "proportional_limitation",
        "antiwindup_cte",
        "alpha",
        "position_following.position_Kp.x",
        "position_following.position_Kp.y",
        "position_following.position_Kp.z",
        "position_following.position_Ki.x",
        "position_following.position_Ki.y",
        "position_following.position_Ki.z",
        "position_following.position_Kd.x",
        "position_following.position_Kd.y",
        "position_following.position_Kd.z",
        "trajectory_following.position_Kp.x",
        "trajectory_following.position_Kp.y",
        "trajectory_following.position_Kp.z",
        "trajectory_following.position_Ki.x",
        "trajectory_following.position_Ki.y",
        "trajectory_following.position_Ki.z",
        "trajectory_following.position_Kd.x",
        "trajectory_following.position_Kd.y",
        "trajectory_following.position_Kd.z",
        "yaw_speed_controller.Kp",
        "yaw_speed_controller.Ki",
        "yaw_speed_controller.Kd",
        // "speed_following.speed_Kp.x",
        // "speed_following.speed_Kp.y",
        // "speed_following.speed_Kp.z",
        // "speed_following.speed_Ki.x",
        // "speed_following.speed_Ki.y",
        // "speed_following.speed_Ki.z",
        // "speed_following.speed_Kd.x",
        // "speed_following.speed_Kd.y",
        // "speed_following.speed_Kd.z",
    };

    UAV_state uav_state_;
    Control_ref control_ref_;
    Control_command control_command_;

    Vector3d speed_limits_;

    UAV_state uav_hover_state_;
  
    bool proportional_limitation_ = false;

  private:
    void declareParameters();

    void resetState();
    void resetReferences();
    void resetCommands();

    void computePositionControl(const double &dt);

    void computeActions(
        geometry_msgs::msg::PoseStamped &pose,
        geometry_msgs::msg::TwistStamped &twist,
        as2_msgs::msg::Thrust &thrust);

    void getOutput(geometry_msgs::msg::PoseStamped &pose_msg,
                   geometry_msgs::msg::TwistStamped &twist_msg,
                   as2_msgs::msg::Thrust &thrust_msg);
  };
};

#endif