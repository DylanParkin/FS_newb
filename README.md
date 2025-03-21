# FS PROJECT

## Dependencies

rclcpp, std_msgs, msgs, ament cmake, ament_lint_auto, ament_lint_common

## Key function

is_target_valid: Checks target location validity, but also recursively tries to find a valid target in the same direction.

## Topics

/target_location, of type msgs/msg/LocationStamped. Provides the time and target location from the path planning algorithm.
/response, of type msgs/msg/Response. Provides information on target position, velocity and acceleration, as well as current vehicle position, velocity and acceleration.
