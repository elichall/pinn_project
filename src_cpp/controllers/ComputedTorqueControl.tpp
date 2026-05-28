namespace Controller {

template <int DOF>
CTC<DOF>::CTC(Model::Robot &perfectModel) : robotModel(perfectModel) {
  Kp = PROPORTIONAL_GAIN;
  Kd = DERIVATIVE_GAIN;
};

template <int DOF>
Eigen::Matrix<double, DOF, 1>
CTC<DOF>::computeControl(const Controller::RobotState<DOF> &state,
                         const Controller::DesiredState<DOF> &dState,
                         const double dt) {

  Eigen::Matrix<double, DOF, 1> e = state.q - dState.dq;
  Eigen::Matrix<double, DOF, 1> edot = state.qdot - dState.dqdot;

  Eigen::Matrix<double, DOF, DOF> M = robotModel.M;
  Eigen::Matrix<double, DOF, 1> C = robotModel.C;
  Eigen::Matrix<double, DOF, 1> G = robotModel.G;

  Eigen::Matrix<double, DOF, 1> tau =
      M * (dState.dqddot - Kd * edot - Kp * e) + C + G;

  return tau;
};

} // namespace Controller
