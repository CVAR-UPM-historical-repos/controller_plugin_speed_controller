/*!*******************************************************************************************
 *  \file       speed_controller_plugin.cpp
 *  \brief      Speed PID controller plugin for the Aerostack framework.
 *  \authors    Miguel Fernández Cortizas
 *              Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *              David Pérez Saura
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

namespace controller_plugin_speed_controller {

void Plugin::ownInitialize() {
  flags_.state_received                        = false;
  flags_.ref_received                          = false;
  flags_.plugin_parameters_read                = false;
  flags_.position_controller_parameters_read   = false;
  flags_.velocity_controller_parameters_read   = false;
  flags_.trajectory_controller_parameters_read = false;
  flags_.yaw_controller_parameters_read        = false;

  speed_limits_ = Eigen::Vector3d::Zero();

  pid_yaw_handler_           = std::make_shared<pid_controller::PIDController>();
  pid_3D_position_handler_   = std::make_shared<pid_controller::PIDController3D>();
  pid_3D_velocity_handler_   = std::make_shared<pid_controller::PIDController3D>();
  pid_3D_trajectory_handler_ = std::make_shared<pid_controller::PIDController3D>();

  tf_handler_ = std::make_shared<as2::tf::TfHandler>(node_ptr_);

  plugin_parameters_to_read_ = std::vector<std::string>(plugin_parameters_list_);
  position_control_parameters_list_ =
      std::vector<std::string>(position_control_parameters_to_read_);
  velocity_control_parameters_list_ =
      std::vector<std::string>(velocity_control_parameters_to_read_);
  trajectory_control_parameters_list_ =
      std::vector<std::string>(trajectory_control_parameters_to_read_);
  yaw_control_parameters_list_ = std::vector<std::string>(yaw_control_parameters_to_read_);

  reset();
  return;
};

bool Plugin::updateParams(const std::vector<std::string> &_params_list) {
  auto result = parametersCallback(node_ptr_->get_parameters(_params_list));
  return result.successful;
};

void Plugin::checkParamList(const std::string &param,
                            std::vector<std::string> &_params_list,
                            bool &_all_params_read) {
  if (find(_params_list.begin(), _params_list.end(), param) != _params_list.end()) {
    // Remove the parameter from the list of parameters to be read
    _params_list.erase(std::remove(_params_list.begin(), _params_list.end(), param),
                       _params_list.end());
  };
  if (_params_list.size() == 0) {
    _all_params_read = true;
  }
};

rcl_interfaces::msg::SetParametersResult Plugin::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason     = "success";

  for (auto &param : parameters) {
    std::string param_name = param.get_name();

    if (param.get_name() == "proportional_limitation") {
      proportional_limitation_ = param.get_value<bool>();
      if (!flags_.plugin_parameters_read) {
        checkParamList(param_name, plugin_parameters_to_read_, flags_.plugin_parameters_read);
      }
    } else if (param.get_name() == "use_bypass") {
      use_bypass_ = param.get_value<bool>();
      if (!flags_.plugin_parameters_read) {
        checkParamList(param_name, plugin_parameters_to_read_, flags_.plugin_parameters_read);
      }
    } else {
      std::string controller    = param_name.substr(0, param_name.find("."));
      std::string param_subname = param_name.substr(param_name.find(".") + 1);
      if (controller == "position_control") {
        updateController3DParameter(pid_3D_position_handler_, param_subname, param);
        if (!flags_.position_controller_parameters_read) {
          checkParamList(param_name, position_control_parameters_to_read_,
                         flags_.position_controller_parameters_read);
        }
      } else if (controller == "velocity_control") {
        updateController3DParameter(pid_3D_velocity_handler_, param_subname, param);
        if (!flags_.velocity_controller_parameters_read) {
          checkParamList(param_name, velocity_control_parameters_to_read_,
                         flags_.velocity_controller_parameters_read);
        }
      } else if (controller == "trajectory_control") {
        updateController3DParameter(pid_3D_trajectory_handler_, param_subname, param);
        if (!flags_.trajectory_controller_parameters_read) {
          checkParamList(param_name, trajectory_control_parameters_to_read_,
                         flags_.trajectory_controller_parameters_read);
        }
      } else if (controller == "yaw_control") {
        updateControllerParameter(pid_yaw_handler_, param_subname, param);
        if (!flags_.yaw_controller_parameters_read) {
          checkParamList(param_name, yaw_control_parameters_to_read_,
                         flags_.yaw_controller_parameters_read);
        }
      }
    }
  }
  return result;
}

void Plugin::updateControllerParameter(
    const std::shared_ptr<pid_controller::PIDController> &_pid_handler,
    const std::string &_parameter_name,
    const rclcpp::Parameter &_param) {
  if (_parameter_name == "reset_integral") {
    _pid_handler->setResetIntegralSaturationFlag(_param.get_value<bool>());
  } else if (_parameter_name == "antiwindup_cte") {
    _pid_handler->setAntiWindup(_param.get_value<double>());
  } else if (_parameter_name == "alpha") {
    _pid_handler->setAlpha(_param.get_value<double>());
  } else if (_parameter_name == "kp") {
    _pid_handler->setGainKp(_param.get_value<double>());
  } else if (_parameter_name == "ki") {
    _pid_handler->setGainKi(_param.get_value<double>());
  } else if (_parameter_name == "kd") {
    _pid_handler->setGainKd(_param.get_value<double>());
  }
  return;
}

void Plugin::updateController3DParameter(
    const std::shared_ptr<pid_controller::PIDController3D> &_pid_handler,
    const std::string &_parameter_name,
    const rclcpp::Parameter &_param) {
  if (_parameter_name == "reset_integral") {
    _pid_handler->setResetIntegralSaturationFlag(_param.get_value<bool>());
  } else if (_parameter_name == "antiwindup_cte") {
    _pid_handler->setAntiWindup(_param.get_value<double>());
  } else if (_parameter_name == "alpha") {
    _pid_handler->setAlpha(_param.get_value<double>());
  } else if (_parameter_name == "kp.x") {
    _pid_handler->setGainKpX(_param.get_value<double>());
  } else if (_parameter_name == "kp.y") {
    _pid_handler->setGainKpY(_param.get_value<double>());
  } else if (_parameter_name == "kp.z") {
    _pid_handler->setGainKpZ(_param.get_value<double>());
  } else if (_parameter_name == "ki.x") {
    _pid_handler->setGainKiX(_param.get_value<double>());
  } else if (_parameter_name == "ki.y") {
    _pid_handler->setGainKiY(_param.get_value<double>());
  } else if (_parameter_name == "ki.z") {
    _pid_handler->setGainKiZ(_param.get_value<double>());
  } else if (_parameter_name == "kd.x") {
    _pid_handler->setGainKdX(_param.get_value<double>());
  } else if (_parameter_name == "kd.y") {
    _pid_handler->setGainKdY(_param.get_value<double>());
  } else if (_parameter_name == "kd.z") {
    _pid_handler->setGainKdZ(_param.get_value<double>());
  }
  return;
}

void Plugin::reset() {
  resetReferences();
  resetState();
  resetCommands();
  pid_yaw_handler_->resetController();
  pid_3D_position_handler_->resetController();
  pid_3D_velocity_handler_->resetController();
  pid_3D_trajectory_handler_->resetController();
  // Info: Yaw rate limit could be set if needed
  // pid_yaw_handler_->setOutputSaturation(yaw_speed_limit_);
}

void Plugin::resetState() {
  uav_state_ = UAV_state();
  return;
}

void Plugin::resetReferences() {
  control_ref_.position_header = uav_state_.position_header;
  control_ref_.position        = uav_state_.position;

  control_ref_.velocity_header = uav_state_.velocity_header;
  control_ref_.velocity        = Eigen::Vector3d::Zero();

  control_ref_.yaw = uav_state_.yaw;
  return;
}

void Plugin::resetCommands() {
  control_command_.velocity_header = std_msgs::msg::Header();
  control_command_.velocity        = Eigen::Vector3d::Zero();
  control_command_.yaw_speed       = 0.0;
  return;
}

void Plugin::updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                         const geometry_msgs::msg::TwistStamped &twist_msg) {
  uav_state_.position_header = pose_msg.header;
  uav_state_.position =
      Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);

  uav_state_.velocity_header = twist_msg.header;
  uav_state_.velocity =
      Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);
  uav_state_.yaw.x() = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);

  flags_.state_received = true;
  return;
};

void Plugin::updateReference(const geometry_msgs::msg::PoseStamped &pose_msg) {
  if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION) {
    control_ref_.position_header = pose_msg.header;
    control_ref_.position = Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y,
                                            pose_msg.pose.position.z);
    flags_.ref_received   = true;
  }

  if ((control_mode_in_.control_mode == as2_msgs::msg::ControlMode::SPEED ||
       control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION) &&
      control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
    control_ref_.yaw.x() = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);
  }

  return;
};

void Plugin::updateReference(const geometry_msgs::msg::TwistStamped &twist_msg) {
  if (control_mode_in_.control_mode == as2_msgs::msg::ControlMode::POSITION) {
    speed_limits_ = Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y,
                                    twist_msg.twist.linear.z);
    pid_3D_position_handler_->setOutputSaturation(speed_limits_);
    pid_3D_velocity_handler_->setOutputSaturation(speed_limits_);
    pid_3D_trajectory_handler_->setOutputSaturation(speed_limits_);
    return;
  }

  if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::SPEED) {
    return;
  }

  control_ref_.velocity_header = twist_msg.header;
  control_ref_.velocity =
      Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z);

  if (control_mode_in_.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    control_ref_.yaw.y() = twist_msg.twist.angular.z;
  }

  flags_.ref_received = true;
  return;
};

void Plugin::updateReference(const trajectory_msgs::msg::JointTrajectoryPoint &traj_msg) {
  if (control_mode_in_.control_mode != as2_msgs::msg::ControlMode::TRAJECTORY) {
    return;
  }

  control_ref_.position =
      Eigen::Vector3d(traj_msg.positions[0], traj_msg.positions[1], traj_msg.positions[2]);

  control_ref_.velocity =
      Eigen::Vector3d(traj_msg.velocities[0], traj_msg.velocities[1], traj_msg.velocities[2]);

  control_ref_.yaw =
      Eigen::Vector3d(traj_msg.positions[3], traj_msg.velocities[3], traj_msg.accelerations[3]);

  flags_.ref_received = true;
  return;
};

bool Plugin::setMode(const as2_msgs::msg::ControlMode &in_mode,
                     const as2_msgs::msg::ControlMode &out_mode) {
  if (in_mode.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    control_mode_in_.control_mode    = in_mode.control_mode;
    control_mode_in_.yaw_mode        = as2_msgs::msg::ControlMode::YAW_ANGLE;
    control_mode_in_.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  } else {
    flags_.ref_received   = false;
    flags_.state_received = false;
    control_mode_in_      = in_mode;
  }

  control_mode_out_ = out_mode;
  reset();

  return true;
};

bool Plugin::computeOutput(const double &dt,
                           geometry_msgs::msg::PoseStamped &pose,
                           geometry_msgs::msg::TwistStamped &twist,
                           as2_msgs::msg::Thrust &thrust) {
  if (!flags_.state_received) {
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "State not received yet");
    return false;
  }

  if (!flags_.plugin_parameters_read) {
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Parameters not read yet");
    return false;
  }

  if (!flags_.position_controller_parameters_read) {
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                         "Parameters for hover controller not read yet");
    return false;
  }

  if (!flags_.ref_received) {
    auto &clk = *node_ptr_->get_clock();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                         "State changed, but ref not recived yet");
    return false;
  }

  resetCommands();

  switch (control_mode_in_.control_mode) {
    case as2_msgs::msg::ControlMode::HOVER: {
      if (!flags_.position_controller_parameters_read) {
        auto &clk = *node_ptr_->get_clock();
        RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                             "Position controller parameters not read yet");
        return false;
      }

      control_command_.velocity_header.frame_id = control_ref_.position_header.frame_id;

      control_command_.velocity =
          pid_3D_position_handler_->computeControl(dt, uav_state_.position, control_ref_.position);
      break;
    }
    case as2_msgs::msg::ControlMode::POSITION: {
      if (!flags_.position_controller_parameters_read) {
        auto &clk = *node_ptr_->get_clock();
        RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                             "Position controller parameters not read yet");
        return false;
      }

      control_command_.velocity_header.frame_id = control_ref_.position_header.frame_id;

      control_command_.velocity =
          pid_3D_position_handler_->computeControl(dt, uav_state_.position, control_ref_.position);
      break;
    }
    case as2_msgs::msg::ControlMode::SPEED: {
      if (use_bypass_) {
        control_command_.velocity_header.frame_id = control_ref_.velocity_header.frame_id;
        control_command_.velocity                 = control_ref_.velocity;
      } else {
        if (!flags_.velocity_controller_parameters_read) {
          auto &clk = *node_ptr_->get_clock();
          RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                               "Velocity controller parameters not read yet");
        }

        control_command_.velocity_header.frame_id = control_ref_.velocity_header.frame_id;

        control_command_.velocity = pid_3D_velocity_handler_->computeControl(
            dt, uav_state_.velocity, control_ref_.velocity);
      }
      break;
    }
    case as2_msgs::msg::ControlMode::TRAJECTORY: {
      if (!flags_.trajectory_controller_parameters_read) {
        auto &clk = *node_ptr_->get_clock();
        RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                             "Trajectory controller parameters not read yet");
        return false;
      }

      control_command_.velocity =
          pid_3D_trajectory_handler_->computeControl(dt, uav_state_.position, control_ref_.position,
                                                     uav_state_.velocity, control_ref_.velocity);
      break;
    }
    default:
      auto &clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown control mode");
      return false;
      break;
  }

  switch (control_mode_in_.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_ANGLE: {
      if (!flags_.yaw_controller_parameters_read) {
        auto &clk = *node_ptr_->get_clock();
        RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), clk, 5000,
                             "Yaw controller parameters not read yet");
        return false;
      }

      double yaw_error = as2::frame::angleMinError(control_ref_.yaw.x(), uav_state_.yaw.x());
      control_command_.yaw_speed = pid_yaw_handler_->computeControl(dt, yaw_error);
      break;
    }
    case as2_msgs::msg::ControlMode::YAW_SPEED: {
      control_command_.yaw_speed = control_ref_.yaw.y();
      break;
    }
    default:
      auto &clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown yaw mode");
      return false;
      break;
  }

  switch (control_mode_out_.reference_frame) {
    case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME: {
      return getOutput(twist, enu_frame_id_);
      break;
    }
    case as2_msgs::msg::ControlMode::BODY_FLU_FRAME: {
      return getOutput(twist, flu_frame_id_);
      break;
    }
    default:
      auto &clk = *node_ptr_->get_clock();
      RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(), clk, 5000, "Unknown reference frame");
      return false;
      break;
  }
  return false;
}

bool Plugin::getOutput(geometry_msgs::msg::TwistStamped &_twist_msg, const std::string &_frame_id) {
  geometry_msgs::msg::TwistStamped control_command_twist;
  control_command_twist.header = control_command_.velocity_header;

  control_command_twist.twist.linear.x = control_command_.velocity.x();
  control_command_twist.twist.linear.y = control_command_.velocity.y();
  control_command_twist.twist.linear.z = control_command_.velocity.z();

  control_command_twist.twist.angular.x = 0;
  control_command_twist.twist.angular.y = 0;
  control_command_twist.twist.angular.z = control_command_.yaw_speed;

  if (control_command_twist.header.frame_id == "") {
    RCLCPP_WARN(node_ptr_->get_logger(), "Control command frame id is empty");
    return false;
  }

  _twist_msg = tf_handler_->convert(control_command_twist, _frame_id);
  return true;
};

}  // namespace controller_plugin_speed_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(controller_plugin_speed_controller::Plugin,
                       controller_plugin_base::ControllerBase)
