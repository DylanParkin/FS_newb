#pragma once
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "vehicle.hpp"

class Target {
 public:
  Target();

  void set_target_data(int sec, int nanosec, double x, double y);

  // Getters
  double get_x();
  double get_y();
  double get_velocity_x();
  double get_velocity_y();
  double get_acceleration_x();
  double get_acceleration_y();
  double get_yaw();
  double get_yaw_rate();
  double get_target_x();
  double get_target_y();
  Vehicle get_vehicle();

  // Main computations
  void is_target_valid(double x, double y, int depth);

  // For recursive target location validity function
  double compute_velocity_mag(double x, double y);
  double compute_acceleration_mag(double x, double y);
  double compute_yaw_rate_mag(double x, double y);

 private:
  std::shared_ptr<Vehicle> vehicle_;
  int sec_;
  int nanosec_;
  float time_;
  float prev_time_;
  double x_;
  double y_;
  double last_valid_target_x_;
  double last_valid_target_y_;
  double velocity_x_;
  double last_valid_velocity_x_;
  double last_valid_velocity_y_;
  double velocity_y_;
  double acceleration_x_;
  double acceleration_y_;
  float yaw_;
  float yaw_rate_;
  float prev_yaw_;
  double valid_target_x_;
  double valid_target_y_;
  double valid_target_velocity_x_;
  double valid_target_velocity_y_;
};