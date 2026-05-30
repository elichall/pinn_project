#include "TrajectoryGenerator.h"
#include <random>

namespace Path {

TrajectoryGenerator::TrajectoryGenerator(const Eigen::Vector3d &start,
                                         const Eigen::Vector3d &end,
                                         double totalTime, int numRandomPoints)
    : duration(totalTime) {
  generatePath(start, end, totalTime, numRandomPoints);
}

void TrajectoryGenerator::generatePath(const Eigen::Vector3d &start,
                                       const Eigen::Vector3d &end,
                                       double totalTime, int numRandomPoints) {
  numWaypoints = 0;
  duration = totalTime;

  // Prevent array overflow
  if (numRandomPoints + 2 > MAX_WAYPOINTS) {
    numRandomPoints = MAX_WAYPOINTS - 2;
  }

  // 1. Setup random number generation for planar noise (X and Y only)
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<> noise(-0.1, 0.1); // +/- 10cm noise

  waypoints[numWaypoints++] = {start, Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero(), 0.0};

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

    waypoints[numWaypoints++] = {basePos, Eigen::Vector3d::Zero(),
                                 Eigen::Vector3d::Zero(), timeAtWaypoint};
  }

  waypoints[numWaypoints++] = {end, Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero(), totalTime};

  // 3. Compute Velocities at Waypoints using Catmull-Rom logic
  // v_i = (p_{i+1} - p_{i-1}) / (t_{i+1} - t_{i-1})
  for (int i = 1; i < numWaypoints - 1; ++i) {
    double dt = waypoints[i + 1].time - waypoints[i - 1].time;
    waypoints[i].velocity =
        (waypoints[i + 1].position - waypoints[i - 1].position) / dt;
  }
  // Note: Start and End velocities remain Zero (starting and ending at rest).

  // 4. Compute Accelerations at Waypoints using central difference of velocities
  // a_i = (v_{i+1} - v_{i-1}) / (t_{i+1} - t_{i-1})
  for (int i = 1; i < numWaypoints - 1; ++i) {
    double dt = waypoints[i + 1].time - waypoints[i - 1].time;
    waypoints[i].acceleration =
        (waypoints[i + 1].velocity - waypoints[i - 1].velocity) / dt;
  }
  // Note: Start and End accelerations remain Zero.
}

int TrajectoryGenerator::getSegmentIndex(double t) const {
  // Clamp time to valid range
  if (t <= 0)
    return 0;
  if (t >= duration)
    return numWaypoints - 2;

  // Find the correct time segment
  for (int i = 0; i < numWaypoints - 1; ++i) {
    if (t >= waypoints[i].time && t <= waypoints[i + 1].time) {
      return i;
    }
  }
  return numWaypoints - 2;
}

Eigen::Vector3d TrajectoryGenerator::getPosition(double t) const {
  if (t <= 0)
    return waypoints[0].position;
  if (t >= duration)
    return waypoints[numWaypoints - 1].position;

  int idx = getSegmentIndex(t);
  const Waypoint &p0 = waypoints[idx];
  const Waypoint &p1 = waypoints[idx + 1];

  // Calculate Quintic Hermite Spline Coefficients
  double dt = p1.time - p0.time;
  double t_local = t - p0.time;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  double dt5 = dt4 * dt;

  Eigen::Vector3d c0 = p0.position;
  Eigen::Vector3d c1 = p0.velocity;
  Eigen::Vector3d c2 = p0.acceleration / 2.0;
  Eigen::Vector3d c3 = (10.0 * (p1.position - p0.position) - (6.0 * p0.velocity + 4.0 * p1.velocity) * dt + 0.5 * (p1.acceleration - 3.0 * p0.acceleration) * dt2) / dt3;
  Eigen::Vector3d c4 = (-15.0 * (p1.position - p0.position) + (8.0 * p0.velocity + 7.0 * p1.velocity) * dt + (1.5 * p0.acceleration - p1.acceleration) * dt2) / dt4;
  Eigen::Vector3d c5 = (6.0 * (p1.position - p0.position) - 3.0 * (p0.velocity + p1.velocity) * dt + 0.5 * (p1.acceleration - p0.acceleration) * dt2) / dt5;

  double t2 = t_local * t_local;
  double t3 = t2 * t_local;
  double t4 = t3 * t_local;
  double t5 = t4 * t_local;

  return c0 + c1 * t_local + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
}

Eigen::Vector3d TrajectoryGenerator::getVelocity(double t) const {
  if (t <= 0 || t >= duration)
    return Eigen::Vector3d::Zero();

  int idx = getSegmentIndex(t);
  const Waypoint &p0 = waypoints[idx];
  const Waypoint &p1 = waypoints[idx + 1];

  double dt = p1.time - p0.time;
  double t_local = t - p0.time;
  
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  double dt5 = dt4 * dt;

  Eigen::Vector3d c1 = p0.velocity;
  Eigen::Vector3d c2 = p0.acceleration / 2.0;
  Eigen::Vector3d c3 = (10.0 * (p1.position - p0.position) - (6.0 * p0.velocity + 4.0 * p1.velocity) * dt + 0.5 * (p1.acceleration - 3.0 * p0.acceleration) * dt2) / dt3;
  Eigen::Vector3d c4 = (-15.0 * (p1.position - p0.position) + (8.0 * p0.velocity + 7.0 * p1.velocity) * dt + (1.5 * p0.acceleration - p1.acceleration) * dt2) / dt4;
  Eigen::Vector3d c5 = (6.0 * (p1.position - p0.position) - 3.0 * (p0.velocity + p1.velocity) * dt + 0.5 * (p1.acceleration - p0.acceleration) * dt2) / dt5;

  double t2 = t_local * t_local;
  double t3 = t2 * t_local;
  double t4 = t3 * t_local;

  return c1 + 2.0 * c2 * t_local + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4;
}

// To get Acceleration, you would take the second derivative of the basis
// functions
Eigen::Vector3d TrajectoryGenerator::getAcceleration(double t) const {
  if (t <= 0 || t >= duration)
    return Eigen::Vector3d::Zero();

  int idx = getSegmentIndex(t);
  const Waypoint &p0 = waypoints[idx];
  const Waypoint &p1 = waypoints[idx + 1];

  double dt = p1.time - p0.time;
  double t_local = t - p0.time;
  
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  double dt5 = dt4 * dt;

  Eigen::Vector3d c2 = p0.acceleration / 2.0;
  Eigen::Vector3d c3 = (10.0 * (p1.position - p0.position) - (6.0 * p0.velocity + 4.0 * p1.velocity) * dt + 0.5 * (p1.acceleration - 3.0 * p0.acceleration) * dt2) / dt3;
  Eigen::Vector3d c4 = (-15.0 * (p1.position - p0.position) + (8.0 * p0.velocity + 7.0 * p1.velocity) * dt + (1.5 * p0.acceleration - p1.acceleration) * dt2) / dt4;
  Eigen::Vector3d c5 = (6.0 * (p1.position - p0.position) - 3.0 * (p0.velocity + p1.velocity) * dt + 0.5 * (p1.acceleration - p0.acceleration) * dt2) / dt5;

  double t2 = t_local * t_local;
  double t3 = t2 * t_local;

  return 2.0 * c2 + 6.0 * c3 * t_local + 12.0 * c4 * t2 + 20.0 * c5 * t3;
}

DesiredPosition TrajectoryGenerator::getDesiredPosition(double t) const {
  DesiredPosition dpos;

  dpos.position = TrajectoryGenerator::getPosition(t);
  dpos.velocity = TrajectoryGenerator::getVelocity(t);
  dpos.acceleration = TrajectoryGenerator::getAcceleration(t);

  return dpos;
}

} // namespace Path
