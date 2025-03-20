#include "target.hpp"

#include <cmath>

Target::Target()
    : sec_(0),
      nanosec_(0),
      time_(0),
      prev_time_(0),
      x_(0.0),
      y_(0.0),
      last_valid_target_x_(0.0),
      last_valid_target_y_(0.0),
      velocity_x_(0.0),
      velocity_y_(0.0),
      last_valid_velocity_x_(0.0),
      last_valid_velocity_y_(0.0),
      acceleration_x_(0.0),
      acceleration_y_(0.0),
      yaw_(0.0),
      yaw_rate_(0.0),
      prev_yaw_(0.0),
      valid_target_x_(0.0),
      valid_target_y_(0.0),
      valid_target_velocity_x_(0.0),
      valid_target_velocity_y_(0.0) {}

void Target::set_target_data(int sec, int nanosec, double x, double y) {
  sec_ = sec;
  nanosec_ = nanosec;
  x_ = x;
  y_ = y;
  time_ = ((sec_ * 1000000000) + nanosec_) / 1000000000.0;

  // Computations
  compute_velocity();
  compute_acceleration();
  compute_yaw();
  compute_yaw_rate();

  // Check if target is valid
  is_target_valid();

  // Update "previous values" with current ones, prior to next iteration
  prev_time_ = time_;
  last_valid_target_x_ = valid_target_x_;
  last_valid_target_y_ = valid_target_y_;
  last_valid_velocity_x_ = valid_target_velocity_x_;
  last_valid_velocity_y_ = valid_target_velocity_y_;
  prev_yaw_ = yaw_;
}

// Getters
double Target::get_x() { return x_; }
double Target::get_y() { return y_; }
double Target::get_velocity_x() { return velocity_x_; }
double Target::get_velocity_y() { return velocity_y_; }
double Target::get_acceleration_x() { return acceleration_x_; }
double Target::get_acceleration_y() { return acceleration_y_; }
float Target::get_yaw() { return yaw_; }
float Target::get_yaw_rate() { return yaw_rate_; }
double Target::get_target_x() { return valid_target_x_; }
double Target::get_target_y() { return valid_target_y_; }

// Computing target attributes using formulae from project brief
void Target::compute_velocity() {
  velocity_x_ = (last_valid_target_x_ - x_) / (prev_time_ - time_);
  velocity_y_ = (last_valid_target_y_ - y_) / (prev_time_ - time_);
}

void Target::compute_acceleration() {
  acceleration_x_ =
      (last_valid_velocity_x_ - velocity_x_) / (prev_time_ - time_);
  acceleration_y_ =
      (last_valid_velocity_y_ - velocity_y_) / (prev_time_ - time_);
}

void Target::compute_yaw() {
  yaw_ = atan2(velocity_y_ - last_valid_velocity_y_,
               velocity_x_ - last_valid_velocity_x_);
}

void Target::compute_yaw_rate() {
  yaw_rate_ = (prev_yaw_ - yaw_) / (prev_time_ - time_);
}

// target = valid if adheres to given kinematic constraints
void Target::is_target_valid() {
  double acc_magnitude =
      sqrt(pow(acceleration_x_, 2) + pow(acceleration_y_, 2));
  double vel_magnitude = sqrt(pow(velocity_x_, 2) + pow(velocity_y_, 2));
  float yaw_rate_magnitude = abs(yaw_rate_);
  if ((acc_magnitude < 2.5) && (vel_magnitude < 10) &&
      (yaw_rate_magnitude < 2.5)) {
    valid_target_x_ = x_;
    valid_target_y_ = y_;
    valid_target_velocity_x_ = velocity_x_;
    valid_target_velocity_y_ = velocity_y_;
  }
}
