#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/publisher_base.hpp>
#include <iostream>
#include <fstream>

using namespace std::chrono_literals;

class MultiPublisherNode : public rclcpp::Node
{
public:
  MultiPublisherNode()
  : Node("multi_publisher_node"), count_(0)
  {
    publisher1_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    log_publisher_info(publisher1_, 1);

    publisher2_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    log_publisher_info(publisher2_, 2);

    publisher3_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    log_publisher_info(publisher3_, 3);

    std::ofstream outfile("handle_debug.log", std::ios::app);
    outfile << "\n";
          
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MultiPublisherNode::timer_callback, this));
  }

private:
  void log_publisher_info(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub, int num)
  {
    auto pub_base = std::static_pointer_cast<rclcpp::PublisherBase>(pub);

    uintptr_t pub_ptr = reinterpret_cast<uintptr_t>(pub.get());
    uintptr_t rcl_pub_ptr = reinterpret_cast<uintptr_t>(pub_base->get_publisher_handle().get());
    int64_t diff_pub_to_rcl = static_cast<int64_t>(rcl_pub_ptr - pub_ptr);

    std::cout << "Publisher #" << num << " object ptr: " << pub_ptr << "\n";
    std::cout << "Publisher #" << num << " rcl_handle: " << rcl_pub_ptr << "\n";
    std::cout << "Diff (Publisher -> rcl): " << diff_pub_to_rcl << "\n\n";

    std::ofstream outfile("handle_debug.log", std::ios::app);
    outfile << "Publisher #" << num << " object ptr: " << pub_ptr << "\t";
    outfile << "Publisher #" << num << " rcl_handle: " << rcl_pub_ptr << "\t";
    outfile << "Diff (Publisher -> rcl): " << diff_pub_to_rcl << "\t";
  }

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello multi world! " + std::to_string(count_++);
    publisher1_->publish(message);
    publisher2_->publish(message);
    publisher3_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Publishing message '%s' on 3 publishers", message.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher3_;

  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
