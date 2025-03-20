#pragma once

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
  float get_yaw();
  float get_yaw_rate();
  double get_target_x();
  double get_target_y();

  // Computations
  void compute_velocity();
  void compute_acceleration();
  void compute_yaw();
  void compute_yaw_rate();
  void is_target_valid();

 private:
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
  double valid_target_x_;  // last valid target location
  double valid_target_y_;  // and the overall target location
  double valid_target_velocity_x_;
  double valid_target_velocity_y_;
};