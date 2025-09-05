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

#include <cstdint>
#include <functional>
#include <memory>
#include <std_msgs/msg/detail/int8__struct.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

using std::placeholders::_1;

class IntegerCounterSubscriber : public rclcpp::Node {

private:
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_;
  std::string topicName = "int_counter";
  rclcpp::QoS qos = rclcpp::QoS(10);

  std::string findParity(std_msgs::msg::Int8::_data_type counter) {
    return (counter % 2 == 0) ? "even" : "odd";
  }

  void subscriber_callback(const std_msgs::msg::Int8 &msg) {
    RCLCPP_INFO(this->get_logger(), "I received '%d'.", msg.data);
    std::cout << "I received " << static_cast<int>(msg.data) << std::flush
              << ". It is an " << findParity(msg.data) << std::flush
              << " number." << std::endl;
  }

public:
  IntegerCounterSubscriber() : Node("integer_counter_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::Int8>(
        topicName, qos,
        std::bind(&IntegerCounterSubscriber::subscriber_callback, this, _1));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntegerCounterSubscriber>());
  rclcpp::shutdown();
  return 0;
}
