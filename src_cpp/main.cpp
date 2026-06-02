#include "ComputedTorqueControl.h"
#include "ControllerInterface.h"
#include "DataLogging.h"
#include "Eigen/src/Core/Matrix.h"
#include "ManipulatorPlant.h"
#include "RobotModel.h"
#include "RobotSensors.h"
#include "SharedMemoryLink.h"
#include "TrajectoryGenerator.h"
#include "helpers.h"
#include "plant/config.h"
#include <Eigen/Core>
#include <cstring>    // For memcpy
#include <fcntl.h>    // For O_CREAT, O_RDWR (File control definitions)
#include <sched.h>    // For SCHED_FIFO and sched_param
#include <sys/mman.h> // For shm_open, mmap, PROT_READ, PROT_WRITE
#include <time.h>     // For clock_nanosleep and CLOCK_MONOTONIC
#include <unistd.h>   // For ftruncate, close

// constants
const double samplingTime = 0.001;
const int DOF = 3;

// data logging thread
// 16384 slots = ~16 seconds of buffer at 1000Hz.
boost::lockfree::spsc_queue<LoggingData<DOF>, boost::lockfree::capacity<16384>>
    telemetryQueue;

// global atomic flag to shut down logging thread when the program closes
std::atomic<bool> simulationRunning{true};

int main() {
  // state data
  Controller::RobotState<DOF> state;
  Path::DesiredPosition desiredPosition;
  Controller::DesiredState<DOF> desiredState;
  Controller::Torque<DOF> torque;

  int mode = 0; // pick up or drop off

  LoggingData<DOF> loggingData;

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
  double systemTime = 0.0; // time since start of sim
  double cycleTime = 0.0;  // time since start of current path cycle

  struct timespec nextWakeTime;   // deterministic time of the next step
  struct timespec actualWakeTime; // for validating deterministic loop
  struct timespec endMathTime;
  clock_gettime(CLOCK_MONOTONIC, &nextWakeTime);
  clock_gettime(CLOCK_MONOTONIC, &actualWakeTime);
  clock_gettime(CLOCK_MONOTONIC, &endMathTime);

  long wakeJitter;
  long executionTime;

  // --- Memory Allocation for IPC ---
  IPC::TelemetryIPC *telemetryIPC = IPC::initTelemetryIPC();
  IPC::PathIPC *pathIPC = IPC::initPathIPC();
  // IPC::CommandIPC *commandIPC = IPC::initCommandIPC();

  // write inital path to shared memory
  sampleAndWritePath(path, pathIPC, telemetryIPC);

  // --- Set Priority for Main Control Loop and Secondary Logger Thread---
  setupRealTimePriority(); // sets main thread priotity to 99

  // create logger thread
  std::thread loggerThread(telemetryLoggerThread, std::ref(simulationRunning),
                           std::ref(telemetryQueue));

  // downgrade the logger thread to standard OS scheduling
  struct sched_param param;
  param.sched_priority = 0;
  pthread_setschedparam(loggerThread.native_handle(), SCHED_OTHER, &param);

  while (true) {
    // --- Time Keeping ---
    // actual time thread wakes up
    wakeJitter = (actualWakeTime.tv_sec - nextWakeTime.tv_sec) * 1000000000L +
                 (actualWakeTime.tv_nsec - nextWakeTime.tv_nsec);

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
    torque.tau =
        activeController->computeControl(state, desiredState, samplingTime);

    // system response (theoretical)
    plant.applyControl(torque.tau, samplingTime);

    // read response from sensors
    sensors.readSensors();
    state = sensors.qEst;

    // check if cycle is complete
    if (cycleTime >= CYCLE_TIME + WAIT_TIME) {
      // change mode of operation
      double objectMass = 10.0; // temp explicity definition
      std::swap(cycleStart, cycleEnd);

      switch (mode) {
      case 0:
        mode = 1;
        // plant picks up or drops an object, changing end effector mass
        plant.pickPlaceObject(
            mode, objectMass); // need to pass in the randomly generated mass
        // model changes its end effector mass to the average of the waste
        // "class"
        model.setMode(mode);

        // generate a point in conveyor belt to go to
        // cycleEnd = generateNewEndPoint
        break;

      case 1:
        mode = 0;
        plant.pickPlaceObject(mode);
        model.setMode(mode);
        break;
      }

      path.generatePath(cycleStart, cycleEnd, CYCLE_TIME, 5);

      sampleAndWritePath(path, pathIPC, telemetryIPC);

      cycleTime = 0.0;
    }

    clock_gettime(CLOCK_MONOTONIC, &endMathTime);

    // record time for physics/control to complete
    executionTime = (endMathTime.tv_sec - actualWakeTime.tv_sec) * 1000000000L +
                    (endMathTime.tv_nsec - actualWakeTime.tv_nsec);

    // --- Write Telemetry ---
    IPC::writeTelemetry(telemetryIPC, state.q.data(), state.qdot.data(),
                        torque.tau.data());

    // --- Data Logging ---
    // Snapshot the state for this cycle
    loggingData.sysTime = systemTime;
    loggingData.wakeJitter = wakeJitter;
    loggingData.executionTime = executionTime;
    loggingData.q = state;
    loggingData.qDes = desiredState;
    loggingData.tau = torque;

    // push data instance to queue
    telemetryQueue.push(loggingData);

    // --- Deterministic System Time Keeping (10000 Hz) ---
    // increment to the next wakeup time
    nextWakeTime.tv_nsec += 1000000;

    // Handle nanosecond overflow into the seconds column
    if (nextWakeTime.tv_nsec >= 1000000000) {
      nextWakeTime.tv_sec += 1;
      nextWakeTime.tv_nsec -= 1000000000;
    }

    // Put the thread to sleep until exactly nextWakeTime
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextWakeTime, NULL);
    clock_gettime(CLOCK_MONOTONIC, &actualWakeTime);
  }

  simulationRunning.store(false);
  loggerThread.join(); // Wait for the logger to finish writing before closing
  return 0;
}
