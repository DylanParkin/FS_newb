#pragma once
#include <iostream>

#include "rclcpp/rclcpp.hpp"

class Vehicle {
 public:
  Vehicle();

  void set_vehicle_target_data(double x, double y, double velocity_x,
                               double velocity_y, double acceleration_x,
                               double acceleration_y, float yaw,
                               float yaw_rate);

  // Getters
  double vehicle_get_x();
  double vehicle_get_y();
  double vehicle_get_velocity_x();
  double vehicle_get_velocity_y();
  double vehicle_get_acceleration_x();
  double vehicle_get_acceleration_y();
  double vehicle_get_yaw();
  double vehicle_get_yaw_rate();
  double vehicle_get_target_x();
  double vehicle_get_target_y();

  // Computations
  void vehicle_compute_location();
  void vehicle_compute_velocity();
  void vehicle_compute_acceleration();
  void vehicle_compute_yaw();
  void vehicle_compute_yaw_rate();

 private:
  double x_;
  double y_;
  double velocity_x_;
  double velocity_y_;
  double acceleration_x_;
  double acceleration_y_;
  float yaw_;
  float yaw_rate_;
};