#include "ComputedTorqueControl.h"
#include "ControllerInterface.h"
#include "Eigen/src/Core/Matrix.h"
#include "ManipulatorPlant.h"
#include "RobotModel.h"
#include "RobotSensors.h"
#include "SharedMemoryLink.h"
#include "TrajectoryGenerator.h"
#include "plant/config.h"
#include <Eigen/Core>
#include <cstring> // For memcpy
#include <fcntl.h> // For O_CREAT, O_RDWR (File control definitions)
#include <fstream>
#include <iostream>
#include <sched.h>    // For SCHED_FIFO and sched_param
#include <sys/mman.h> // For shm_open, mmap, PROT_READ, PROT_WRITE
#include <time.h>     // For clock_nanosleep and CLOCK_MONOTONIC
#include <unistd.h>   // For ftruncate, close
#include <vector>

// struct LogFrame {
//   double time;
//   double q_des[3];
//   double qdot_des[3];
//   double q_act[3];
//   double qdot_act[3];
//   double tau[3];
// };

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

void sampleAndWritePath(Path::TrajectoryGenerator &path, IPC::PathIPC *pathIPC, IPC::TelemetryIPC *telemetryIPC) {
  std::vector<double> pathX(MAX_PATH_POINTS);
  std::vector<double> pathY(MAX_PATH_POINTS);
  std::vector<double> pathZ(MAX_PATH_POINTS);
  double pathStep = CYCLE_TIME / MAX_PATH_POINTS;
  double pathTime = 0.0;
  for (int i = 0; i < MAX_PATH_POINTS; i++) {
    Eigen::Vector3d pos = path.getPosition(pathTime);
    pathX[i] = pos[0];
    pathY[i] = pos[1];
    pathZ[i] = pos[2];
    pathTime += pathStep;
  }
  IPC::writePath(pathX.data(), pathY.data(), pathZ.data(), MAX_PATH_POINTS, pathIPC, telemetryIPC);
}

// constants
const double samplingTime = 0.001;
const int DOF = 3;

int main() {
  // data
  Controller::RobotState<DOF> state;
  Path::DesiredPosition desiredPosition;
  Controller::DesiredState<DOF> desiredState;

  // initalizations
  Plant::Robot plant;
  Sensors::JointSensors sensors(plant);
  sensors.readSensors();
  state = sensors.qEst;
  Model::Robot model(state);

  Controller::CTC<DOF> ctc(model);

  Eigen::Vector3d cycleStart = INITAL_STATE_EPS;
  Eigen::Vector3d cycleEnd = FINAL_STATE_EPS;
  Path::TrajectoryGenerator path = {cycleStart, cycleEnd, CYCLE_TIME, 5};

  Controller::ControllerInterface<DOF> *activeController = &ctc;

  // time keeping
  double systemTime = 0.0;
  double cycleTime = 0.0;
  struct timespec nextWakeTime;
  clock_gettime(CLOCK_MONOTONIC, &nextWakeTime);

  // std::vector<LogFrame> logBuffer;
  // logBuffer.reserve(10000); // Pre-allocate for up to 10 seconds at 1000Hz

  // --- Memory Allocation for IPC ---
  IPC::TelemetryIPC *telemetryIPC = IPC::initTelemetryIPC();
  IPC::PathIPC *pathIPC = IPC::initPathIPC();

  // --- Write Path to Shared Memory ---
  sampleAndWritePath(path, pathIPC, telemetryIPC);

  // --- Set Priority for Main Control Loop ---
  setupRealTimePriority();

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

    // system response (theoretical)
    plant.applyControl(tau, samplingTime);

    // read response from sensors
    sensors.readSensors();
    state = sensors.qEst;

    // --- Write Telemetry ---
    IPC::writeTelemetry(telemetryIPC, state.q.data(), state.qdot.data(),
                        tau.data());

    // --- Check if Cycle is Complete ---
    if (cycleTime >= CYCLE_TIME + WAIT_TIME) {
      std::swap(cycleStart, cycleEnd);

      path.generatePath(cycleStart, cycleEnd, CYCLE_TIME, 5);

      sampleAndWritePath(path, pathIPC, telemetryIPC);

      cycleTime = 0.0;
    }

    // // --- Logging ---
    // LogFrame frame;
    // frame.time = systemTime;
    // for (int i = 0; i < 3; ++i) {
    //   frame.q_des[i] = desiredState.dq[i];
    //   frame.qdot_des[i] = desiredState.dqdot[i];
    //   frame.q_act[i] = state.q[i];
    //   frame.qdot_act[i] = state.qdot[i];
    //   frame.tau[i] = tau[i];
    // }
    // logBuffer.push_back(frame);

    // // Break after 2 seconds to dump telemetry
    // if (systemTime >= 15.0) {
    //   break;
    // }

    // --- Deterministic System Time Keeping (10000 Hz) ---
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

  // // --- Dump Log to CSV ---
  // std::ofstream logFile("telemetry_log.csv");
  // if (logFile.is_open()) {
  //   logFile <<
  //   "time,q_des_0,q_des_1,q_des_2,qdot_des_0,qdot_des_1,qdot_des_2,"
  //           << "q_act_0,q_act_1,q_act_2,qdot_act_0,qdot_act_1,qdot_act_2,"
  //           << "tau_0,tau_1,tau_2\n";
  //   for (const auto &f : logBuffer) {
  //     logFile << f.time << ",";
  //     for (int i = 0; i < 3; ++i)
  //       logFile << f.q_des[i] << ",";
  //     for (int i = 0; i < 3; ++i)
  //       logFile << f.qdot_des[i] << ",";
  //     for (int i = 0; i < 3; ++i)
  //       logFile << f.q_act[i] << ",";
  //     for (int i = 0; i < 3; ++i)
  //       logFile << f.qdot_act[i] << ",";
  //     for (int i = 0; i < 3; ++i) {
  //       logFile << f.tau[i];
  //       if (i < 2)
  //         logFile << ",";
  //     }
  //     logFile << "\n";
  //   }
  //   logFile.close();
  //   std::cout << "Telemetry saved to telemetry_log.csv" << std::endl;
  // }

  return 0;
}
