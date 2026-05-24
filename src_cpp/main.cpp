#include "ComputedTorqueControl.h"
#include "ControllerInterface.h"
#include "ManipulatorPlant.h"
#include "RobotModel.h"
#include "TrajectoryGenerator.h"
#include "plant/config.h"
#include <Eigen/Core>

// constants
const int DOF = 3;
const double samplingTime = 0.001;

// initalizations
Plant::Robot plant;
Model::Robot model;

Controller::RobotState state;
Controller::DesiredState desiredState;

Controller::CTC ctc = {model};

Path::TrajectoryGenerator path = {INITAL_STATE_EPS, FINAL_STATE_EPS, CYCLE_TIME,
                                  5};

Controller::ControllerInterface *activeController = &ctc;

// time keeping
double systemTime = 0.0;
double cycleTime = 0.0;

int main() {
  systemTime += samplingTime;
  cycleTime += samplingTime;

  // get this time's desiredState

  // compute controll effort
  Eigen::VectorXd tau =
      activeController->computeControl(state, desiredState, samplingTime);

  // system response
  plant.update(tau, samplingTime);
}
