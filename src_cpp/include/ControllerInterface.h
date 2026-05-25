#pragma once
#include "constants.h"
#include <Eigen/Core>

namespace Controller {

struct RobotState {
  Eigen::Matrix<double, DOF, 1> q;
  Eigen::Matrix<double, DOF, 1> qdot;
};

struct DesiredState {
  Eigen::Matrix<double, DOF, 1> dq;
  Eigen::Matrix<double, DOF, 1> dqdot;
  Eigen::Matrix<double, DOF, 1> dqddot;
};

class ControllerInterface {
public:
  // default deconstructor
  virtual ~ControllerInterface() = default;
  // enforce any controller type to have a function which follows this templet
  virtual Eigen::Matrix<double, DOF, 1>
  computeControl(const RobotState &currentState, const DesiredState &refState,
                 const double dt) = 0;
};

} // namespace Controller
