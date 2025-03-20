#include "vehicle_node.hpp"

VehicleNode::VehicleNode() : Node("vehicle_node") {
  subscription_ = this->create_subscription<msgs::msg::LocationStamped>(
      "target_location", 10,
      std::bind(&VehicleNode::topic_callback, this, std::placeholders::_1));

  /* Example publisher setup (currently commented out)
  publisher_ = this->create_publisher<std_msgs::msg::String>("vehicle_info",
  10);
  */

  target_ = std::make_shared<Target>();
}

void VehicleNode::topic_callback(
    const msgs::msg::LocationStamped::SharedPtr msg) {
  int sec = msg->header.stamp.sec;
  int nanosec = msg->header.stamp.nanosec;
  double x = msg->location.x;
  double y = msg->location.y;

  /* Example logging (uncomment if needed)
  RCLCPP_INFO(this->get_logger(),
              "Received target location: sec=%d, nanosec=%d, x=%.2f, y=%.2f",
              sec, nanosec, x, y);
  */

  target_->set_target_data(sec, nanosec, x, y);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<VehicleNode>());

  rclcpp::shutdown();
  return 0;
}