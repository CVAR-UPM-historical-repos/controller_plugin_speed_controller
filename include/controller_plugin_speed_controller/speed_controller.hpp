
/*!*******************************************************************************************
 *  \file       speed_controller.hpp
 *  \brief      This file contains the implementation of the Speed PID
 *controller with speed output. \authors    Miguel Fernández Cortizas Pedro
 *Arias Pérez David Pérez Saura Rafael Pérez Seguí
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

#ifndef __SC_CONTROLLER_H__
#define __SC_CONTROLLER_H__

// #include <Eigen/src/Core/Matrix.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>

namespace speed_controller {
using Vector3d = Eigen::Vector3d;

struct UAV_state {
  Vector3d pos;
  Vector3d vel;
  tf2::Quaternion rot;
};

struct Control_ref {
  Vector3d pos;
  Vector3d vel;
  Vector3d yaw;
};

struct Control_command {
  Vector3d vel;
  Vector3d yaw;
};

class SpeedController {
public:
  SpeedController();
  ~SpeedController(){};

public:
  bool setParameter(const std::string &param, const double &value);
  bool getParameter(const std::string &param, double &value);
  bool isParameter(const std::string &param);
  bool setParametersList(const std::vector<std::pair<std::string, double>> &parameter_list);
  bool getParametersList(std::vector<std::string> &param_list);
  std::vector<std::pair<std::string, double>> getParametersMap();

  Vector3d computePositionControl(const UAV_state &state, const Control_ref &ref, const double &dt);

  Vector3d computeTrayectoryControl(const UAV_state &state,
                                    const Control_ref &ref,
                                    const double &dt);

  Vector3d computeSpeedControl(const UAV_state &state, const Control_ref &ref, const double &dt);

  double computeYawSpeed(const double &yaw_angle_state,
                         const double &yaw_angle_ref,
                         const double &dt);

  void resetError();

private:
  Eigen::Vector3d position_accum_error_      = Eigen::Vector3d::Zero();
  Eigen::Vector3d traj_position_accum_error_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d speed_accum_error_         = Eigen::Vector3d::Zero();
  double yaw_accum_error_                    = 0.0;

  std::unordered_map<std::string, double> parameters_ = {
      {"antiwindup_cte", 5.0},
      {"alpha", 0.1},
      // {"reset_integral_flag", 0.0},
      {"position_following.position_Kp.x", 1.0},
      {"position_following.position_Kp.y", 1.0},
      {"position_following.position_Kp.z", 1.0},
      {"position_following.position_Ki.x", 0.0},
      {"position_following.position_Ki.y", 0.0},
      {"position_following.position_Ki.z", 0.0},
      {"position_following.position_Kd.x", 0.0},
      {"position_following.position_Kd.y", 0.0},
      {"position_following.position_Kd.z", 0.0},
      {"trajectory_following.position_Kp.x", 1.0},
      {"trajectory_following.position_Kp.y", 1.0},
      {"trajectory_following.position_Kp.z", 1.0},
      {"trajectory_following.position_Ki.x", 0.01},
      {"trajectory_following.position_Ki.y", 0.01},
      {"trajectory_following.position_Ki.z", 0.01},
      {"trajectory_following.position_Kd.x", 0.0},
      {"trajectory_following.position_Kd.y", 0.0},
      {"trajectory_following.position_Kd.z", 0.0},
      {"yaw_speed_controller.Kp", 1.0},
      {"yaw_speed_controller.Ki", 1.0},
      {"yaw_speed_controller.Kd", 1.0},
      // {"speed_following.speed_Kp.x", 1.0},
      // {"speed_following.speed_Kp.y", 1.0},
      // {"speed_following.speed_Kp.z", 1.0},
      // {"speed_following.speed_Ki.x", 0.0},
      // {"speed_following.speed_Ki.y", 0.0},
      // {"speed_following.speed_Ki.z", 0.0},
      // {"speed_following.speed_Kd.x", 0.0},
      // {"speed_following.speed_Kd.y", 0.0},
      // {"speed_following.speed_Kd.z", 0.0},
  };

  std::vector<std::string> parameters_list_ = {
      "antiwindup_cte", "alpha",
      // "reset_integral_flag",
      "position_following.position_Kp.x", "position_following.position_Kp.y",
      "position_following.position_Kp.z", "position_following.position_Ki.x",
      "position_following.position_Ki.y", "position_following.position_Ki.z",
      "position_following.position_Kd.x", "position_following.position_Kd.y",
      "position_following.position_Kd.z", "trajectory_following.position_Kp.x",
      "trajectory_following.position_Kp.y", "trajectory_following.position_Kp.z",
      "trajectory_following.position_Ki.x", "trajectory_following.position_Ki.y",
      "trajectory_following.position_Ki.z", "trajectory_following.position_Kd.x",
      "trajectory_following.position_Kd.y", "trajectory_following.position_Kd.z",
      "yaw_speed_controller.Kp", "yaw_speed_controller.Ki", "yaw_speed_controller.Kd",
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

  float antiwindup_cte_       = 1.0f;
  double alpha_               = 0.1;
  double reset_integral_flag_ = 0.0;

  Eigen::Matrix3d traj_Kp_lin_mat_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d traj_Ki_lin_mat_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d traj_Kd_lin_mat_ = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d position_Kp_lin_mat_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d position_Ki_lin_mat_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d position_Kd_lin_mat_ = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d speed_Kp_lin_mat_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d speed_Ki_lin_mat_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d speed_Kd_lin_mat_ = Eigen::Matrix3d::Identity();

  Eigen::Vector3d yaw_ang_mat_ = Eigen::Vector3d::Identity();

private:
  void updateGains_();
};
};  // namespace speed_controller

#endif
