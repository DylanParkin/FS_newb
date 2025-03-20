#include "target.hpp"

Target::Target() : sec_(0), nanosec_(0), x_(0.0), y_(0.0) {}

void Target::set_target_data(int sec, int nanosec, double x, double y) {
  sec_ = sec;
  nanosec_ = nanosec;
  x_ = x;
  y_ = y;
}

// Getter function implementations here

void Target::compute_velocity() {}

void Target::compute_acceleration() {}

void Target::compute_yaw() {}

void Target::compute_yaw_rate() {}
