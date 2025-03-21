#include "vehicle.hpp"

#include <algorithm>

Vehicle::Vehicle()
    : x_(0.0),
      y_(0.0),
      velocity_x_(0.0),
      velocity_y_(0.0),
      acceleration_x_(0.0),
      acceleration_y_(0.0),
      yaw_(0.0),
      yaw_rate_(0.0) {}

void Vehicle::set_vehicle_target_data(double x, double y, double velocity_x,
                                      double velocity_y, double acceleration_x,
                                      double acceleration_y, float yaw,
                                      float yaw_rate) {
  acceleration_x_ = std::clamp(acceleration_x, -2.5,
                               2.5);  // needs to account for overall magnitude
  acceleration_y_ = std::clamp(acceleration_x, -2.5, 2.5);
  velocity_x_ = std::clamp(velocity_x, -10.0, 10.0);
  velocity_y_ = std::clamp(velocity_y, -10.0, 10.0);
}

void Vehicle::vehicle_compute_location() {}

void Vehicle::vehicle_compute_velocity() {}

void Vehicle::vehicle_compute_acceleration() {}

void Vehicle::vehicle_compute_yaw() {}

void Vehicle::vehicle_compute_yaw_rate() {}

// Getters
double Vehicle::vehicle_get_x() { return x_; }
double Vehicle::vehicle_get_y() { return y_; }
double Vehicle::vehicle_get_velocity_x() { return velocity_x_; }
double Vehicle::vehicle_get_velocity_y() { return velocity_y_; }
double Vehicle::vehicle_get_acceleration_x() { return acceleration_x_; }
double Vehicle::vehicle_get_acceleration_y() { return acceleration_y_; }
double Vehicle::vehicle_get_yaw() { return yaw_; }
double Vehicle::vehicle_get_yaw_rate() { return yaw_rate_; }
double Vehicle::vehicle_get_target_x() { return x_; }
double Vehicle::vehicle_get_target_y() { return y_; }