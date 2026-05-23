#pragma once
#include "ComputedTorqueControl.h"
#include "Eigen/src/Core/Matrix.h"
#include <Eigen/Core>

namespace Controller {

CTC::CTC(Plant::Robot &perfectModel) : robotModel(perfectModel) {
  Kp = PROPORTIONAL_GAIN;
  Kd = DERIVATIVE_GAIN;
};

Eigen::VectorXd CTC::computeControl(const Controller::RobotState &state,
                                    const Controller::DesiredState &dState,
                                    const double dt) {
  Eigen::VectorXd e = state.q - dState.dq;
  Eigen::VectorXd edot = state.qdot - dState.dqdot;

  Eigen::MatrixXd M = robotModel.M;
  Eigen::VectorXd C = robotModel.C;
  Eigen::VectorXd G = robotModel.G;

  Eigen::VectorXd tau = M * (dState.dqddot + Kd * edot + Kp * e) + C + G;

  return tau;
};

} // namespace Controller
