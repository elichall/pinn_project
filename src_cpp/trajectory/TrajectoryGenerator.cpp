#include "TrajectoryGenerator.h"
#include <algorithm>
#include <cmath>
#include <random>

namespace Path {

TrajectoryGenerator::TrajectoryGenerator(const Eigen::Vector3d &start,
                                         const Eigen::Vector3d &end,
                                         double totalTime, int numRandomPoints)
    : duration(totalTime) {
  // 1. Setup random number generation for planar noise (X and Y only)
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> noise(-0.1, 0.1); // +/- 10cm noise

  waypoints.push_back({start, Eigen::Vector3d::Zero(), 0.0});

  // 2. Generate random intermediate points
  // To keep the path progressing forward, we linearly interpolate from start
  // to end, and add noise perpendicularly or around that point.
  for (int i = 1; i <= numRandomPoints; ++i) {
    double fraction = static_cast<double>(i) / (numRandomPoints + 1);

    Eigen::Vector3d basePos = start + fraction * (end - start);

    // Add planar noise (Z remains strictly 0)
    basePos.x() += noise(gen);
    basePos.y() += noise(gen);
    basePos.z() = 0.0;

    // Time is strictly proportional to distance assuming constant linear
    // velocity
    double timeAtWaypoint = fraction * totalTime;

    waypoints.push_back({basePos, Eigen::Vector3d::Zero(), timeAtWaypoint});
  }

  waypoints.push_back({end, Eigen::Vector3d::Zero(), totalTime});

  // 3. Compute Velocities at Waypoints using Catmull-Rom logic
  // v_i = (p_{i+1} - p_{i-1}) / (t_{i+1} - t_{i-1})
  for (size_t i = 1; i < waypoints.size() - 1; ++i) {
    double dt = waypoints[i + 1].time - waypoints[i - 1].time;
    waypoints[i].velocity =
        (waypoints[i + 1].position - waypoints[i - 1].position) / dt;
  }
  // Note: Start and End velocities remain Zero (starting and ending at rest).
}

int TrajectoryGenerator::getSegmentIndex(double t) const {
  // Clamp time to valid range
  if (t <= 0)
    return 0;
  if (t >= duration)
    return waypoints.size() - 2;

  // Find the correct time segment
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    if (t >= waypoints[i].time && t <= waypoints[i + 1].time) {
      return i;
    }
  }
  return waypoints.size() - 2;
}

Eigen::Vector3d TrajectoryGenerator::getPosition(double t) const {
  if (t <= 0)
    return waypoints.front().position;
  if (t >= duration)
    return waypoints.back().position;

  int idx = getSegmentIndex(t);
  const Waypoint &p0 = waypoints[idx];
  const Waypoint &p1 = waypoints[idx + 1];

  // Normalize time for this specific segment (s goes from 0.0 to 1.0)
  double dt = p1.time - p0.time;
  double s = (t - p0.time) / dt;

  // Cubic Hermite Spline Basis Functions
  double h00 = 2 * s * s * s - 3 * s * s + 1;
  double h10 = s * s * s - 2 * s * s + s;
  double h01 = -2 * s * s * s + 3 * s * s;
  double h11 = s * s * s - s * s;

  // Calculate position
  return h00 * p0.position + h10 * dt * p0.velocity + h01 * p1.position +
         h11 * dt * p1.velocity;
}

Eigen::Vector3d TrajectoryGenerator::getVelocity(double t) const {
  if (t <= 0 || t >= duration)
    return Eigen::Vector3d::Zero();

  int idx = getSegmentIndex(t);
  const Waypoint &p0 = waypoints[idx];
  const Waypoint &p1 = waypoints[idx + 1];

  double dt = p1.time - p0.time;
  double s = (t - p0.time) / dt;

  // Derivative of Basis Functions (with respect to s, multiplied by 1/dt for
  // chain rule)
  double dh00 = (6 * s * s - 6 * s) / dt;
  double dh10 = (3 * s * s - 4 * s + 1); // dt cancels out
  double dh01 = (-6 * s * s + 6 * s) / dt;
  double dh11 = (3 * s * s - 2 * s); // dt cancels out

  return dh00 * p0.position + dh10 * p0.velocity + dh01 * p1.position +
         dh11 * p1.velocity;
}

// To get Acceleration, you would take the second derivative of the basis
// functions
Eigen::Vector3d TrajectoryGenerator::getAcceleration(double t) const {
  // Leaving this math implementation for you to practice taking the next
  // derivative!
  return Eigen::Vector3d::Zero();
}

} // namespace Path
