#pragma once
#include "ComputedTorqueControl.h"
#include "Eigen/src/Core/Matrix.h"
#include <Eigen/Core>

namespace Controller {

CTC::CTC(Model::Robot &perfectModel) : robotModel(perfectModel) {
  Kp = PROPORTIONAL_GAIN;
  Kd = DERIVATIVE_GAIN;
};

Eigen::Matrix<double, DOF, 1>
CTC::computeControl(const Controller::RobotState &state,
                    const Controller::DesiredState &dState, const double dt) {
  Eigen::Matrix<double, DOF, 1> e = state.q - dState.dq;
  Eigen::Matrix<double, DOF, 1> edot = state.qdot - dState.dqdot;

  Eigen::Matrix<double, DOF, DOF> M = robotModel.M;
  Eigen::Matrix<double, DOF, 1> C = robotModel.C;
  Eigen::Matrix<double, DOF, 1> G = robotModel.G;

  Eigen::Matrix<double, DOF, 1> tau =
      M * (dState.dqddot + Kd * edot + Kp * e) + C + G;

  return tau;
};

} // namespace Controller
