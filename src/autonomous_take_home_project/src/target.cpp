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
      valid_target_velocity_y_(0.0),
      vehicle_(std::make_shared<Vehicle>()) {}

void Target::set_target_data(int sec, int nanosec, double x, double y) {
  sec_ = sec;
  nanosec_ = nanosec;
  x_ = x;
  y_ = y;
  time_ = ((sec_ * 1000000000) + nanosec_) / 1000000000.0;

  // Initial computations
  float dt = time_ - prev_time_;
  velocity_x_ = double((x_ - last_valid_target_x_) / dt);
  velocity_y_ = double((y_ - last_valid_target_y_) / dt);
  acceleration_x_ = double((velocity_x_ - last_valid_velocity_x_) / dt);
  acceleration_y_ = double((velocity_y_ - last_valid_velocity_y_) / dt);
  yaw_ = double(atan2(velocity_y_ - last_valid_velocity_y_,
                      velocity_x_ - last_valid_velocity_x_));
  yaw_rate_ = double((yaw_ - prev_yaw_) / dt);

  // Seeing validity
  is_target_valid(x_, y_, 0);

  vehicle_->set_vehicle_target_data(x_, y_, velocity_x_, velocity_y_,
                                    acceleration_x_, acceleration_y_, yaw_,
                                    yaw_rate_);

  // Update "previous values" with current ones, prior to next iteration
  prev_time_ = time_;
  last_valid_target_x_ = valid_target_x_;
  last_valid_target_y_ = valid_target_y_;
  last_valid_velocity_x_ = valid_target_velocity_x_;
  last_valid_velocity_y_ = valid_target_velocity_y_;
  prev_yaw_ = yaw_;
}

// Getters
double Target::get_x() { return std::round(x_ * 100.0) / 100.0; }
double Target::get_y() { return std::round(y_ * 100.0) / 100.0; }
double Target::get_velocity_x() {
  return std::round(velocity_x_ * 100.0) / 100.0;
}
double Target::get_velocity_y() {
  return std::round(velocity_y_ * 100.0) / 100.0;
}
double Target::get_acceleration_x() {
  return std::round(acceleration_x_ * 100.0) / 100.0;
}
double Target::get_acceleration_y() {
  return std::round(acceleration_y_ * 100.0) / 100.0;
}
double Target::get_yaw() { return std::round(yaw_ * 100.0) / 100.0; }
double Target::get_yaw_rate() { return std::round(yaw_rate_ * 100.0) / 100.0; }
double Target::get_target_x() {
  return std::round(valid_target_x_ * 100.0) / 100.0;
}
double Target::get_target_y() {
  return std::round(valid_target_y_ * 100.0) / 100.0;
}
Vehicle Target::get_vehicle() { return *vehicle_; }

// Used in target location validity func
double Target::compute_velocity_mag(double x, double y) {
  float dt = time_ - prev_time_;
  double vel_x = (x - last_valid_target_x_) / dt;
  double vel_y = (y - last_valid_target_y_) / dt;
  return double(sqrt(pow(vel_x, 2) + pow(vel_y, 2)));
}

double Target::compute_acceleration_mag(double x, double y) {
  float dt = time_ - prev_time_;
  double vel_x = (x - last_valid_target_x_) / dt;
  double vel_y = (y - last_valid_target_y_) / dt;
  double acc_x = (vel_x - last_valid_velocity_x_) / dt;
  double acc_y = (vel_y - last_valid_velocity_y_) / dt;
  return double(sqrt(pow(acc_x, 2) + pow(acc_y, 2)));
}

double Target::compute_yaw_rate_mag(double x, double y) {
  float dt = time_ - prev_time_;
  double vel_x = (x - last_valid_target_x_) / dt;
  double vel_y = (y - last_valid_target_y_) / dt;
  double yaw = double(
      atan2(vel_y - last_valid_velocity_y_, vel_x - last_valid_velocity_x_));
  return double(abs((yaw - prev_yaw_) / dt));
}

// Recursively calls until finds a valid target in general direction of target
// location
void Target::is_target_valid(double x, double y, int depth) {
  if (depth > 20) {  // Prevent infinite recursion
    RCLCPP_WARN(rclcpp::get_logger("Target"), "Max recursion depth reached.");
    return;
  }
  float dt = time_ - prev_time_;
  if (compute_velocity_mag(x, y) < 10.0 &&
      compute_acceleration_mag(x, y) < 2.5 &&
      compute_yaw_rate_mag(x, y) < 2.5) {
    valid_target_x_ = x;
    valid_target_y_ = y;
    valid_target_velocity_x_ = (x - last_valid_target_x_) / dt;
    valid_target_velocity_y_ = (y - last_valid_target_y_) / dt;
  } else {
    double vehicle_pos_x = vehicle_->vehicle_get_x();
    double vehicle_pos_y = vehicle_->vehicle_get_y();
    double weighted_x = 0.9 * vehicle_pos_x + 0.1 * x;
    double weighted_y = 0.9 * vehicle_pos_y + 0.1 * y;
    is_target_valid(weighted_x, weighted_y, depth + 1);
  }
}
