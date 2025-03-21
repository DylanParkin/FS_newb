#include "vehicle_node.hpp"

VehicleNode::VehicleNode() : Node("vehicle_node") {
  subscription_ = this->create_subscription<msgs::msg::LocationStamped>(
      "target_location", 10,
      std::bind(&VehicleNode::topic_callback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<msgs::msg::Response>("response", 10);

  target_ = std::make_shared<Target>();
}

void VehicleNode::topic_callback(
    const msgs::msg::LocationStamped::SharedPtr msg) {
  int sec = msg->header.stamp.sec;
  int nanosec = msg->header.stamp.nanosec;
  double x = msg->location.x;
  double y = msg->location.y;

  // "target_" will process the data
  target_->set_target_data(sec, nanosec, x, y);

  // Message:
  auto target_msg = msgs::msg::Response();

  target_msg.target.position.x = target_->get_x();
  target_msg.target.position.y = target_->get_y();
  target_msg.target.position.z = 0.0;

  target_msg.target.velocity.x = target_->get_velocity_x();
  target_msg.target.velocity.y = target_->get_velocity_y();
  target_msg.target.velocity.z = 0.0;

  target_msg.target.acceleration.x = target_->get_acceleration_x();
  target_msg.target.acceleration.y = target_->get_acceleration_y();
  target_msg.target.acceleration.z = 0.0;

  target_msg.target.angle = target_->get_yaw();
  target_msg.target.anglular_velocity = target_->get_yaw_rate();

  /*
  target_msg.vehicle.position.x = target_->get_vehicle().vehicle_get_x();
  target_msg.vehicle.position.y = target_->get_vehicle().vehicle_get_y();
  target_msg.vehicle.position.z = 0.0;

  target_msg.vehicle.velocity.x =
      target_->get_vehicle().vehicle_get_velocity_x();
  target_msg.vehicle.velocity.y =
      target_->get_vehicle().vehicle_get_velocity_y();
  target_msg.vehicle.velocity.z = 0.0;

  target_msg.vehicle.acceleration.x =
      target_->get_vehicle().vehicle_get_acceleration_x();
  target_msg.vehicle.acceleration.y =
      target_->get_vehicle().vehicle_get_acceleration_y();
  target_msg.vehicle.acceleration.z = 0.0;
  */

  target_msg.lvtl.x = target_->get_target_x();
  target_msg.lvtl.y = target_->get_target_y();
  target_msg.lvtl.z = 0.0;

  target_msg.target_location.x = target_->get_target_x();
  target_msg.target_location.y = target_->get_target_y();
  target_msg.target_location.z = 0.0;

  publisher_->publish(target_msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<VehicleNode>());

  rclcpp::shutdown();
  return 0;
}