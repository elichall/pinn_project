#include "ControllerInterface.h"
#include "ManipulatorPlant.h"
#include <Eigen/Core>

namespace Sensors {

class JointSensors {
public:
  JointSensors(Plant::Robot &sensedPlant);
  ~JointSensors() = default;

  Controller::RobotState<3> qEst;

  // read sensors
  void readSensors();

private:
  Plant::Robot &plant;

  Controller::RobotState<3> qMeas;

  // filters for future simulated noise
  void filter();
};

} // namespace Sensors
