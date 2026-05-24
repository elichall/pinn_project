#include <Eigen/Core>

// Physical Parameters
const double LINK_1_MASS = 10.0; // kg

const double LINK_2_MASS = 4.0;          // kg
const double LINK_2_LENGTH = 0.8;        // m
const double LINK_2_INERTIAL_MASS = 0.8; // kg-m^2

const double END_MASS = 0.5; // kg

const double CYCLE_TIME = 0.5; // sec

const Eigen::Vector3d INITAL_STATE_EPS = {1.0, 0.5, 0.0};        // m
const Eigen::Vector3d INITAL_STATE_DOT_EPS = {0.0f, 0.0f, 0.0f}; // m/s

const Eigen::Vector3d FINAL_STATE_EPS = {1.0, -0.7, 0.0};
