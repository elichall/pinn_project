#pragma once

#include <Eigen/Core>
#include <array>

namespace Path {

constexpr int MAX_WAYPOINTS = 20;

struct Waypoint {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity; // Needed for continuous splines
  double time;              // Master clock time when the robot should be here
};

struct DesiredPosition {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
};

class TrajectoryGenerator {
public:
  // Generates a path with 'numRandomPoints' between start and end.
  TrajectoryGenerator(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                      double totalTime, int numRandomPoints);

  ~TrajectoryGenerator() = default;

  void generatePath(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                    double totalTime, int numRandomPoints);

  // Callable at any time t. Returns Cartesian [X, Y, Z]
  Eigen::Vector3d getPosition(double t) const;
  Eigen::Vector3d getVelocity(double t) const;
  Eigen::Vector3d getAcceleration(double t) const;
  DesiredPosition getDesiredPosition(double t) const;

private:
  std::array<Waypoint, MAX_WAYPOINTS> waypoints;
  int numWaypoints = 0;
  double duration;

  // Helper function to find which segment of the spline we are currently in
  int getSegmentIndex(double t) const;
};

} // namespace Path
