
#ifndef __SC_CONTROLLER_H__
#define __SC_CONTROLLER_H__

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <unordered_map>

namespace speed_controller
{
  using Vector3d = Eigen::Vector3d;

  struct UAV_state
  {
    Vector3d pos;
    Vector3d vel;
    Eigen::Matrix3d rot;
  };

  struct Control_ref
  {
    Vector3d pos;
    Vector3d vel;
    Vector3d acc;
    Vector3d yaw;
  };

  class SpeedController
  {
  public:
    SpeedController();
    ~SpeedController(){};
  };
};

#endif
