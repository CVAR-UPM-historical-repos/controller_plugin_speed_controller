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