
/*!*******************************************************************************************
 *  \file       speed_controller.hpp
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

#ifndef __SC_CONTROLLER_H__
#define __SC_CONTROLLER_H__

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <unordered_map>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace speed_controller
{
  using Vector3d = Eigen::Vector3d;

  struct UAV_state
  {
    Vector3d pos;
    Vector3d vel;
    tf2::Quaternion rot;
  };

  struct Control_ref
  {
    Vector3d pos;
    Vector3d vel;
    Vector3d yaw;
  };

  struct Control_command
  {
    Vector3d vel;
    Vector3d yaw;
  };

  class SpeedController
  {
  public:
    SpeedController();
    ~SpeedController(){};

  public:
    bool setParameter(const std::string &param, const double &value);
    bool getParameter(const std::string &param, double &value);
    bool isParameter(const std::string &param);
    bool setParametersList(const std::vector<std::pair<std::string, double>> &parameter_list);
    std::vector<std::pair<std::string, double>> getParametersList();

    Vector3d computePositionControl(
        const UAV_state &state_,
        const Control_ref &ref_,
        const double &dt);

    Vector3d computeTrayectoryControl(
      const UAV_state &state_,
      const Control_ref &ref,
      const double &dt);

    double computeYawSpeed(
        const double &yaw_angle_state,
        const double &yaw_angle_ref,
        const double &dt);

  private:
    std::unordered_map<std::string, double> parameters_ = {
        {"traj_Kp", 1.0},
        {"pos_Kp", 1},
        {"pos_Kd", 0.1},
    };

    float traj_Kp_ = 0.1;

    float pos_Kp_ = 1;
    float pos_Kd_ = 0.1;

  private:
    void updateGains_();

  };
};

#endif
