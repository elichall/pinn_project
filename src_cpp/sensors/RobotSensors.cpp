#include "RobotSensors.h"

namespace Sensors {

JointSensors::JointSensors(Plant::Robot &sensedPlant) : plant(sensedPlant) {};

void JointSensors::readSensors() {
  qMeas.q = plant.q;
  qMeas.qdot = plant.qdot;

  filter();
}

void JointSensors::filter() { qEst = qMeas; }

} // namespace Sensors
