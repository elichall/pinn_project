#pragma once
#include "ControllerInterface.h"
#include "RobotModel.h"
#include "config.h"
#include <Eigen/Core>

namespace Controller {

template <int DOF>
class CTC : public ControllerInterface<DOF> {
public:
  // constructor and destructor
  CTC(Model::Robot &robotModel);
  ~CTC() = default;

  Eigen::Matrix<double, DOF, 1>
  computeControl(const Controller::RobotState<DOF> &state,
                 const Controller::DesiredState<DOF> &dState, const double dt) override;

private:
  double Kp;
  double Kd;

  Model::Robot &robotModel;
};

} // namespace Controller

#include "ComputedTorqueControl.tpp"
