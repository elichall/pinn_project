#pragma once

#include "SharedMemoryLink.h"
#include "TrajectoryGenerator.h"

#include "config.h"
#include <iostream>
#include <array>

// Helper functions for main

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

void sampleAndWritePath(Path::TrajectoryGenerator &path, IPC::PathIPC *pathIPC,
                        IPC::TelemetryIPC *telemetryIPC) {
  static std::array<double, MAX_PATH_POINTS> pathX;
  static std::array<double, MAX_PATH_POINTS> pathY;
  static std::array<double, MAX_PATH_POINTS> pathZ;
  double pathStep = CYCLE_TIME / MAX_PATH_POINTS;
  double pathTime = 0.0;
  for (int i = 0; i < MAX_PATH_POINTS; i++) {
    Eigen::Vector3d pos = path.getPosition(pathTime);
    pathX[i] = pos[0];
    pathY[i] = pos[1];
    pathZ[i] = pos[2];
    pathTime += pathStep;
  }
  IPC::writePath(pathX.data(), pathY.data(), pathZ.data(), MAX_PATH_POINTS,
                 pathIPC, telemetryIPC);
}
