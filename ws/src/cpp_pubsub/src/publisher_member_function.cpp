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
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/publisher_base.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      1ms, std::bind(&MinimalPublisher::timer_callback, this));

    auto publisher_base = std::static_pointer_cast<rclcpp::PublisherBase>(publisher_);

// pointer address:
// uintptr_t node_ptr = reinterpret_cast<uintptr_t>(this);
// uintptr_t pub_ptr = reinterpret_cast<uintptr_t>(publisher_.get());
// uintptr_t rcl_pub_ptr = reinterpret_cast<uintptr_t>(publisher_base->get_publisher_handle().get());

// int64_t diff_node_to_rcl = static_cast<int64_t>(rcl_pub_ptr - node_ptr);
// int64_t diff_pub_to_rcl = static_cast<int64_t>(rcl_pub_ptr - pub_ptr);

// std::ofstream outfile("handle_debug.log", std::ios::app);
// outfile << "Node: " << node_ptr << "\t";
// outfile << "Publisher: " << pub_ptr << "\t";
// outfile << "rcl_publisher_t: " << rcl_pub_ptr << "\t";
// outfile << "Diff (Node -> rcl): " << diff_node_to_rcl << "\t";
// outfile << "Diff (Publisher -> rcl): " << diff_pub_to_rcl << "\n";
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Node::SharedPtr node1 = std::make_shared<MinimalPublisher>();
    executor.add_node(node1);
    // rclcpp::Node::SharedPtr node2 = std::make_shared<MinimalPublisher>();
    // executor.add_node(node2);
    executor.spin();
  rclcpp::shutdown();
  return 0;
}


// #include <chrono>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <rclcpp/publisher_base.hpp>
// #include <iostream>
// #include <fstream>

// using namespace std::chrono_literals;

// class MultiPublisherNode : public rclcpp::Node
// {
// public:
//   MultiPublisherNode()
//   : Node("multi_publisher_node"), count_(0)
//   {
//     publisher1_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//     log_publisher_info(publisher1_, 1);

//     publisher2_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//     log_publisher_info(publisher2_, 2);

//     publisher3_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//     log_publisher_info(publisher3_, 3);

//     std::ofstream outfile("handle_debug.log", std::ios::app);
//     outfile << "\n";
          
//     timer_ = this->create_wall_timer(
//       500ms, std::bind(&MultiPublisherNode::timer_callback, this));
//   }

// private:
//   void log_publisher_info(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub, int num)
//   {
//     auto pub_base = std::static_pointer_cast<rclcpp::PublisherBase>(pub);

//     uintptr_t pub_ptr = reinterpret_cast<uintptr_t>(pub.get());
//     uintptr_t rcl_pub_ptr = reinterpret_cast<uintptr_t>(pub_base->get_publisher_handle().get());
//     int64_t diff_pub_to_rcl = static_cast<int64_t>(rcl_pub_ptr - pub_ptr);

//     std::cout << "Publisher #" << num << " object ptr: " << pub_ptr << "\n";
//     std::cout << "Publisher #" << num << " rcl_handle: " << rcl_pub_ptr << "\n";
//     std::cout << "Diff (Publisher -> rcl): " << diff_pub_to_rcl << "\n\n";

//     std::ofstream outfile("handle_debug.log", std::ios::app);
//     outfile << "Publisher #" << num << " object ptr: " << pub_ptr << "\t";
//     outfile << "Publisher #" << num << " rcl_handle: " << rcl_pub_ptr << "\t";
//     outfile << "Diff (Publisher -> rcl): " << diff_pub_to_rcl << "\t";
//   }

//   void timer_callback()
//   {
//     auto message = std_msgs::msg::String();
//     message.data = "Hello multi world! " + std::to_string(count_++);
//     publisher1_->publish(message);
//     publisher2_->publish(message);
//     publisher3_->publish(message);

//     RCLCPP_INFO(this->get_logger(), "Publishing message '%s' on 3 publishers", message.data.c_str());
//   }

//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher1_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher3_;

//   rclcpp::TimerBase::SharedPtr timer_;
//   size_t count_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<MultiPublisherNode>();
//   // rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
