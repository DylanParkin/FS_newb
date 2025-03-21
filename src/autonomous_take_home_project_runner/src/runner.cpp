#include <chrono>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <thread>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "msgs/msg/location_stamped.hpp"
#include "msgs/msg/response.hpp"

class LocationPublisher : public rclcpp::Node {
 public:
  LocationPublisher(const std::string& csv_file)
      : Node("location_publisher"), file_path_(csv_file) {
    publisher_ = this->create_publisher<msgs::msg::LocationStamped>(
        "target_location", 10);
    subscriber_ = this->create_subscription<msgs::msg::Response>(
        "response", 10,
        std::bind(&LocationPublisher::response_callback, this,
                  std::placeholders::_1));
    load_csv_data();

    timer_ = this->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int64_t>(locations_[0].time)),
        std::bind(&LocationPublisher::publish_location, this));
  }

  ~LocationPublisher() {
    RCLCPP_INFO(this->get_logger(),
                "node shutting down, changes r coming thru");
  }

 private:
  void load_csv_data() {
    std::ifstream file(file_path_);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s",
                   file_path_.c_str());
      return;
    }

    std::string line;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      uint32_t time, x, y;
      if (ss >> time && ss.ignore() && ss >> x && ss.ignore() && ss >> y) {
        locations_.push_back({time, x, y});
      }
    }
    file.close();
    // RCLCPP_INFO(this->get_logger(), "Loaded %zu locations",
    // locations_.size());
  }

  void publish_location() {
    // if (index_ >= locations_.size()) {
    //     RCLCPP_INFO(this->get_logger(), "All locations published.");
    //     return;
    // }

    auto msg = msgs::msg::LocationStamped();
    // todo
    msg.header.stamp.sec = locations_[index_].time / 1000000000;
    msg.header.stamp.nanosec = locations_[index_].time % 1000000000;
    msg.header.frame_id = "map";
    msg.location.x = locations_[index_].x;
    msg.location.y = locations_[index_].y;
    msg.location.z = 0.0;

    publisher_->publish(msg);

    // RCLCPP_INFO(this->get_logger(), "Published location: [%.2f, %.2f]",
    // msg.location.x, msg.location.y);

    index_++;

    if (index_ >= locations_.size()) {
      RCLCPP_INFO(this->get_logger(), "All locations published.");
      RCLCPP_INFO(
          this->get_logger(),
          "Sorry the verifier is not working atm. Please check back later");
      timer_->cancel();
      return;
    }

    // RCLCPP_INFO(this->get_logger(), "Time stamp: %li s, %li ns",
    // msg.header.stamp.sec, msg.header.stamp.nanosec);

    uint64_t temp_time;
    if (index_ > 0) {
      temp_time = locations_[index_].time - locations_[index_ - 1].time;
    } else {
      temp_time = locations_[index_].time;
    }
    auto millis = std::chrono::nanoseconds(static_cast<int64_t>(temp_time));
    change_timer_period(millis);
  }

  void change_timer_period(std::chrono::nanoseconds new_period) {
    // Cancel the existing timer
    if (timer_) {
      timer_->cancel();
    }

    // Create a new timer with the updated period
    timer_ = this->create_wall_timer(
        new_period, std::bind(&LocationPublisher::publish_location, this));
    // RCLCPP_INFO(this->get_logger(), "Timer period changed to %li ms",
    // new_period.count());
  }

  void response_callback(const msgs::msg::Response msg) {
    RCLCPP_INFO(this->get_logger(), "Received response");
    // rclcpp::shutdown();
  }

  rclcpp::Publisher<msgs::msg::LocationStamped>::SharedPtr publisher_;
  rclcpp::Subscription<msgs::msg::Response>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string file_path_;

  struct LocationData {
    uint64_t time, x, y;
  };
  std::vector<LocationData> locations_;
  size_t index_ = 0;
  size_t responces_seen = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Usage: ros2 run <package_name> <node_name> <csv_file>");
    return 1;
  }
  auto node = std::make_shared<LocationPublisher>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
