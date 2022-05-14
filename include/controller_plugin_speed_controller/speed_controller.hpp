
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
        // {"pos_Kp", 0.1},
        // {"pos_Kv", 0.1},
    };

    float traj_Kp_ = 0.1;

    float pos_Kp_ = 0.1;
    float pos_Kv_ = 0.1;

  private:
    void updateGains_();

  };
};

#endif
