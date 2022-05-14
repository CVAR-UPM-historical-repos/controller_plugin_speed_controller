/*!*******************************************************************************************
 *  \file       speed_controller.cpp
 *  \brief      This file contains the implementation of the Speed PID controller with speed output.
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

#include "speed_controller.hpp"

namespace speed_controller
{
  SpeedController::SpeedController()
  {
    updateGains_();
  };

  bool SpeedController::setParameter(const std::string &param, const double &value)
  {
    if (parameters_.count(param) == 1)
    {
      parameters_[param] = value;
      updateGains_();
      return true;
    }
    return false;
  };

  bool SpeedController::getParameter(const std::string &param, double &value)
  {
    if (parameters_.count(param) == 1)
    {
      value = parameters_[param];
      return true;
    }
    return false;
  };

  bool SpeedController::isParameter(const std::string &param)
  {
    return parameters_.count(param) == 1;
  };

  bool SpeedController::setParametersList(const std::vector<std::pair<std::string, double>> &parameter_list)
  {
    for (auto &param : parameter_list)
    {
      if (parameters_.count(param.first) == 1)
      {
        parameters_[param.first] = param.second;
      }
      else
      {
        return false;
      }
    }
    updateGains_();
    return true;
  };

  std::vector<std::pair<std::string, double>> SpeedController::getParametersList()
  {
    std::vector<std::pair<std::string, double>> list;
    for (auto &param : parameters_)
    {
      list.push_back({param.first, param.second});
    }
    return list;
  };

  void SpeedController::updateGains_()
  {
    traj_Kp_ = parameters_["traj_Kp"];

    // pos_Kp_ = parameters_["pos_Kp"];
    // pos_Kv_ = parameters_["pos_Kv"];

    return;
  };

  // Return velocity control command
  Vector3d SpeedController::computePositionControl(
      const UAV_state &state_,
      const Control_ref &ref,
      const double &dt)
  {
    Vector3d vel_ref = ref.vel;
    Vector3d pos_ref = ref.pos;
    Vector3d pos_state = state_.pos;
    Vector3d vel_state = state_.vel;
    Vector3d pos_err = pos_ref - pos_state;
    Vector3d vel_err = vel_ref - vel_state;

    return pos_Kp_ * pos_err + pos_Kv_ * vel_err; // TODO
  };

  Vector3d SpeedController::computeTrayectoryControl(
      const UAV_state &state_,
      const Control_ref &ref,
      const double &dt)
  {
    return ref.vel + traj_Kp_ * (ref.pos - state_.pos);
  };

  // Return yaw speed in rad/s to reach a reference yaw angle
  double SpeedController::computeYawSpeed(
        const double &yaw_angle_state,
        const double &yaw_angle_ref,
        const double &dt)
  {
    double yaw_angle_ref_wrap = yaw_angle_ref;
    if (yaw_angle_ref_wrap < -M_PI)
    {
        yaw_angle_ref_wrap += 2.0 * M_PI;
    }
    else if (yaw_angle_ref_wrap > M_PI)
    {
        yaw_angle_ref_wrap -= 2.0 * M_PI;
    }

    double yaw_angle_diff = 0.0;

    yaw_angle_diff = yaw_angle_ref_wrap - yaw_angle_state;

    if (yaw_angle_diff < -M_PI)
    {
        return yaw_angle_diff + 2.0 * M_PI;
    }
    else if (yaw_angle_diff > M_PI)
    {
        return yaw_angle_diff - 2.0 * M_PI;
    }
    return yaw_angle_diff;
  };

}