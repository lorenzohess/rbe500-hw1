// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/detail/int8__struct.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
private:
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_;
  std::string topicName = "int_counter";
  std::chrono::milliseconds timerPeriodms = 1000ms;
  rclcpp::QoS qos = rclcpp::QoS(10);
  size_t count_;

  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback() {
    auto message = std_msgs::msg::Int8();
    message.data = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing counter: '%d'", message.data);
    publisher_->publish(message);
  }

public:
  MinimalPublisher() : Node( "integer_counter_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Int8>(topicName, qos);
    timer_ = this->create_wall_timer(
        timerPeriodms, std::bind(&MinimalPublisher::timer_callback, this));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
