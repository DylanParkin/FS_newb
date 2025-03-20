#ifndef VEHICLE_NODE_HPP_
#define VEHICLE_NODE_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "msgs/msg/location_stamped.hpp"
#include "msgs/msg/response.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "target.hpp"

class VehicleNode : public rclcpp::Node {
 public:
  VehicleNode();

 private:
  void topic_callback(const msgs::msg::LocationStamped::SharedPtr msg);

  std::shared_ptr<Target> target_;
  rclcpp::Subscription<msgs::msg::LocationStamped>::SharedPtr subscription_;
};

#endif  // VEHICLE_NODE_HPP_