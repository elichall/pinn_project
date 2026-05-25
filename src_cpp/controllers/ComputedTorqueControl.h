#pragma once
#include "ControllerInterface.h"
#include "RobotModel.h"
#include "config.h"
#include "constants.h"
#include <Eigen/Core>

namespace Controller {

class CTC : public ControllerInterface {
public:
  // constructor and destructor
  CTC(Model::Robot &robotModel);
  ~CTC() = default;

  Eigen::Matrix<double, DOF, 1>
  computeControl(const Controller::RobotState &state,
                 const Controller::DesiredState &dState, const double dt);

private:
  double Kp;
  double Kd;

  Model::Robot &robotModel;
};

} // namespace Controller

#include "ComputedTorqueControl.tpp"
