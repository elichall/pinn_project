#include "ComputedTorqueControl.h"
#include "ControllerInterface.h"
#include "ManipulatorPlant.h"
#include "RobotModel.h"
#include "TrajectoryGenerator.h"
#include "plant/config.h"
#include <Eigen/Core>

// constants
const double samplingTime = 0.001;

// initalizations
Plant::Robot plant;
Model::Robot model;

Controller::RobotState state;
Path::DesiredPosition desiredPosition;
Controller::DesiredState desiredState;

Controller::CTC ctc(model);

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
  desiredPosition = path.getDesiredPosition(cycleTime);
  desiredState = model.invKinematics(desiredPosition);

  // compute control effort
  Eigen::VectorXd tau =
      activeController->computeControl(state, desiredState, samplingTime);

// system response
#pragma omp parallel sections
  {
#pragma omp section
    {
      plant.update(tau, samplingTime);
    }
#pragma omp section
    {
      model.update(tau, samplingTime);
    }
  }

  state.q = plant.q; // temp perfect model and sensing assumptions
  state.qdot = plant.qdot;
}
