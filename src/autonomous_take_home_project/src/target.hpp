#pragma once

class Target {
 public:
  Target();

  void update(int sec, int nanosec, double x, double y);
  void compute_velocity();
  void compute_acceleration();
  void compute_yaw();
  void compute_yaw_rate();
  void is_target_valid();

  int get_sec() const;
  int get_nanosec() const;
  double get_x() const;
  double get_y() const;

 private:
  int sec_;
  int nanosec_;
  int time_;
  int prev_time_;
  double x_;
  double y_;
  double last_valid_x_;
  double last_valid_y_;
  double velocity_x_;
  double prev_velocity_x_;
  double prev_velocity_y_;
  double velocity_y_;
  double acceleration_x_;
  double acceleration_y_;
  float yaw_;
  float yaw_rate_;
  double valid_target_x_;           // last valid target location
  double valid_target_location_y_;  // and the overall target location
};