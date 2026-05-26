#include "ComputedTorqueControl.h"
#include "ControllerInterface.h"
#include "ManipulatorPlant.h"
#include "RobotModel.h"
#include "RobotSensors.h"
#include "SharedMemoryLink.h"
#include "TrajectoryGenerator.h"
#include "plant/config.h"
#include <Eigen/Core>
#include <cstring> // For memcpy
#include <fcntl.h> // For O_CREAT, O_RDWR (File control definitions)
#include <iostream>
#include <sched.h>    // For SCHED_FIFO and sched_param
#include <sys/mman.h> // For shm_open, mmap, PROT_READ, PROT_WRITE
#include <time.h>     // For clock_nanosleep and CLOCK_MONOTONIC
#include <unistd.h>   // For ftruncate, close

// functions
void setupRealTimePriority() {
  struct sched_param param;
  param.sched_priority = 99; // 99 is the absolute highest POSIX priority

  // SCHED_FIFO means "First In, First Out". Once this thread starts running,
  // the OS is not allowed to interrupt it for any other normal program.
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    // This will fail if you don't run the executable with 'sudo'.
    std::cerr << "WARNING: Failed to set real-time priority. Run with sudo!"
              << std::endl;
  }
}

// constants
const double samplingTime = 0.001;
const int DOF = 3;

// data
Controller::RobotState<DOF> state;
Path::DesiredPosition desiredPosition;
Controller::DesiredState<DOF> desiredState;

// initalizations
Plant::Robot plant;
Sensors::JointSensors sensors(plant);
Model::Robot model(state);

Controller::CTC<DOF> ctc(model);

Path::TrajectoryGenerator path = {INITAL_STATE_EPS, FINAL_STATE_EPS, CYCLE_TIME,
                                  5};

Controller::ControllerInterface<DOF> *activeController = &ctc;

// time keeping
double systemTime = 0.0;
double cycleTime = 0.0;
struct timespec nextWakeTime;

int main() {
  // Time keeping for strict 10000 Hz loop
  clock_gettime(CLOCK_MONOTONIC, &nextWakeTime);

  // --- Memory Allocation for IPC ---
  IPC::TelemetryIPC *telemetryIPC = IPC::initTelemetryIPC();
  IPC::PathIPC *pathIPC = IPC::initPathIPC();

  // --- Write Path to Shared Memory ---
  IPC::writePath(pathIPC, telemetryIPC);

  // --- Set Priority for Main Control Loop ---
  while (true) {

    // --- Phyics and Control ---
    // update the model's values
    model.update();

    // physics time keeping
    systemTime += samplingTime;
    cycleTime += samplingTime;

    // get this time's desiredState
    desiredPosition = path.getDesiredPosition(cycleTime);
    desiredState = model.invKinematics(desiredPosition, samplingTime);

    // compute control effort
    Eigen::VectorXd tau =
        activeController->computeControl(state, desiredState, samplingTime);

    // system response (theoretical step)
    plant.applyControl(tau, samplingTime);

    // read response from sensors
    sensors.readSensors();
    state = sensors.qEst;

    // --- Write Telemetry ---
    IPC::writeTelemetry(telemetryIPC, state.q.data(), state.qdot.data(), tau.data());

    // --- System Time Keeping ---
    // increment to the next wakeup time
    nextWakeTime.tv_nsec += 1000000;

    // Handle nanosecond overflow into the seconds column
    if (nextWakeTime.tv_nsec >= 1000000000) {
      nextWakeTime.tv_sec += 1;
      nextWakeTime.tv_nsec -= 1000000000;
    }

    // Put the thread to sleep until exactly nextWakeTime.
    // Using TIMER_ABSTIME ensures phase drift doesn't accumulate over time.
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextWakeTime, NULL);
  }

  return 0;
}
