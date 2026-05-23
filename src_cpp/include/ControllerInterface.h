#pragma once
#include <Eigen/Core>

namespace Controller {

struct RobotState {
  Eigen::Vector3d q;
  Eigen::Vector3d qdot;
};

struct DesiredState {
  Eigen::VectorXd dq;
  Eigen::VectorXd dqdot;
  Eigen::VectorXd dqddot;
};

class ControllerInterface {
public:
  // default deconstructor
  virtual ~ControllerInterface() = default;
  // enforce any controller type to have a function which follows this templet
  virtual Eigen::VectorXd computeControl(const RobotState &currentState,
                                         const DesiredState &refState,
                                         const double dt) = 0;
};

} // namespace Controller
