#include "msgs/msg/Response.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class VehicleNode : public rclcpp::Node {
 public:
  VehicleNode() : Node("vehicle_node") {
    subscription_ = this->create_subscription<msgs::msg::Response>(
        "target_location", 10,
        std::bind(&VehicleNode::topic_callback, this, std::placeholders::_1));

    /* publishes to Response topic
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("vehicle_info", 10);
        */
    target_ = std::make_shared<Target>();
  }

 private:
  void topic_callback(const msgs::msg::Response::SharedPtr msg) {
    int sec = msg->header.stamp.sec;
    int nanosec = msg->header.stamp.nanosec;
    double x = msg->location.x;
    double y = msg->location.y;

    /* Print out the values (or process them as needed)
    RCLCPP_INFO(this->get_logger(),
                "Received target location: sec=%d, nanosec=%d, x=%.2f, y=%.2f",
                sec, nanosec, x, y); */
  }

  rclcpp::Subscription<msgs::msg::Response>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<VehicleNode>());

  rclcpp::shutdown();
  return 0;
}