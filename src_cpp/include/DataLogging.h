#pragma once

#include "ControllerInterface.h"
#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>
#include <chrono>
#include <fstream>
#include <thread>

template <int DOF> struct LoggingData {
  double sysTime;     // time since sim start
  long wakeJitter;    // ns off deterministic 1000Hz that sample time
  long executionTime; // ns taken to do the math
  Controller::RobotState<DOF> q;      // actual joint space state
  Controller::DesiredState<DOF> qDes; // desired joint space state
  Controller::Torque<DOF> tau;        // control effort
};

void telemetryLoggerThread(std::atomic<bool> &simulationRunning,
                           boost::lockfree::spsc_queue<
                               LoggingData<3>, boost::lockfree::capacity<16384>>
                               &telemetryQueue) {

  std::ofstream csvFile("telemetry_log.csv");
  csvFile << "sysTime,wakeJitter,executionTime,"
          << "q1,q2,q3,qdot1,qdot2,qdot3,"
          << "dq1,dq2,dq3,dqdot1,dqdot2,dqdot3,dqddot1,dqddot2,dqddot3,"
          << "tau1,tau2,tau3\n";

  LoggingData<3> dataBuffer;

  while (simulationRunning.load()) {
    // Sleep for 100ms (10Hz). The 1000Hz loop will have pushed ~100 items
    // into the queue while we were sleeping.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Pop items until the queue is completely empty
    while (telemetryQueue.pop(dataBuffer)) {
      // Write to SSD
      csvFile << dataBuffer.sysTime << "," << dataBuffer.wakeJitter << ","
              << dataBuffer.executionTime << "," 
              << dataBuffer.q.q(0) << "," << dataBuffer.q.q(1) << "," << dataBuffer.q.q(2) << "," 
              << dataBuffer.q.qdot(0) << "," << dataBuffer.q.qdot(1) << "," << dataBuffer.q.qdot(2) << "," 
              << dataBuffer.qDes.dq(0) << "," << dataBuffer.qDes.dq(1) << "," << dataBuffer.qDes.dq(2) << "," 
              << dataBuffer.qDes.dqdot(0) << "," << dataBuffer.qDes.dqdot(1) << "," << dataBuffer.qDes.dqdot(2) << "," 
              << dataBuffer.qDes.dqddot(0) << "," << dataBuffer.qDes.dqddot(1) << "," << dataBuffer.qDes.dqddot(2) << "," 
              << dataBuffer.tau.tau(0) << "," << dataBuffer.tau.tau(1) << "," << dataBuffer.tau.tau(2) << "\n";
    }

    // Flush the buffer to disk at 10Hz so external scripts can read cleanly
    csvFile.flush();
  }

  // Final drain right before shutdown
  while (telemetryQueue.pop(dataBuffer)) {
    csvFile << dataBuffer.sysTime << "," << dataBuffer.wakeJitter << ","
            << dataBuffer.executionTime << "," 
            << dataBuffer.q.q(0) << "," << dataBuffer.q.q(1) << "," << dataBuffer.q.q(2) << "," 
            << dataBuffer.q.qdot(0) << "," << dataBuffer.q.qdot(1) << "," << dataBuffer.q.qdot(2) << "," 
            << dataBuffer.qDes.dq(0) << "," << dataBuffer.qDes.dq(1) << "," << dataBuffer.qDes.dq(2) << "," 
            << dataBuffer.qDes.dqdot(0) << "," << dataBuffer.qDes.dqdot(1) << "," << dataBuffer.qDes.dqdot(2) << "," 
            << dataBuffer.qDes.dqddot(0) << "," << dataBuffer.qDes.dqddot(1) << "," << dataBuffer.qDes.dqddot(2) << "," 
            << dataBuffer.tau.tau(0) << "," << dataBuffer.tau.tau(1) << "," << dataBuffer.tau.tau(2) << "\n";
  }
  csvFile.close();
}
