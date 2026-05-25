#pragma once
#include <Eigen/Core>

namespace Controller {

template <int DOF>
struct RobotState {
  Eigen::Matrix<double, DOF, 1> q;
  Eigen::Matrix<double, DOF, 1> qdot;
};

template <int DOF>
struct DesiredState {
  Eigen::Matrix<double, DOF, 1> dq;
  Eigen::Matrix<double, DOF, 1> dqdot;
  Eigen::Matrix<double, DOF, 1> dqddot;
};

template <int DOF>
class ControllerInterface {
public:
  // default deconstructor
  virtual ~ControllerInterface() = default;
  // enforce any controller type to have a function which follows this templet
  virtual Eigen::Matrix<double, DOF, 1>
  computeControl(const RobotState<DOF> &currentState, const DesiredState<DOF> &refState,
                 const double dt) = 0;
};

} // namespace Controller
