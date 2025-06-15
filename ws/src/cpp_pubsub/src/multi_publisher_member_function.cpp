#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/publisher_base.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <random>

using namespace std::chrono_literals;

class MultiPublisherNode : public rclcpp::Node
{
public:
  MultiPublisherNode()
  : Node("multi_publisher_node"), count_(0), publisher_index_(0)
  {
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MultiPublisherNode::timer_callback, this));

    std::thread([this]() {
      while (publisher_index_ < max_publishers_) {
        int delay = rand_range(1, 5);
        RCLCPP_INFO(this->get_logger(), "Wait %d for pub #%d ...", delay, publisher_index_ + 1);
        std::this_thread::sleep_for(std::chrono::seconds(delay));
        create_and_log_publisher();
      }
    }).detach();
  }

private:
  static int rand_range(int min, int max)
  {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(min, max);
    return dis(gen);
  }

  void create_and_log_publisher()
  {
    ++publisher_index_;
    std::string topic = "topic_" + std::to_string(publisher_index_);
    auto pub = this->create_publisher<std_msgs::msg::Bool>(topic, 10);
    publishers_.push_back(pub);
    log_publisher_info(pub, publisher_index_);
  }

  void log_publisher_info(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & pub, int num)
  {
    auto pub_base = std::static_pointer_cast<rclcpp::PublisherBase>(pub);

    uintptr_t node_ptr = reinterpret_cast<uintptr_t>(this);
    uintptr_t pub_ptr = reinterpret_cast<uintptr_t>(pub.get());
    uintptr_t rcl_pub_ptr = reinterpret_cast<uintptr_t>(pub_base->get_publisher_handle().get());

    int64_t diff_node_to_rcl = static_cast<int64_t>(rcl_pub_ptr - node_ptr);
    int64_t diff_pub_to_rcl = static_cast<int64_t>(rcl_pub_ptr - pub_ptr);

    std::string log_line =
      std::to_string(num) + ":\t" +
      "Node: " + std::to_string(node_ptr) +
      "\tPublisher: " + std::to_string(pub_ptr) +
      "\trcl_publisher_t: " + std::to_string(rcl_pub_ptr) +
      "\tDiff (Node -> rcl): " + std::to_string(diff_node_to_rcl) +
      "\tDiff (Publisher -> rcl): " + std::to_string(diff_pub_to_rcl);

    std::cout << log_line << std::endl;

    std::ofstream outfile("handle_debug.log", std::ios::app);
    outfile << log_line << std::endl;
  }

  void timer_callback()
  {
    if (publishers_.empty()) return;

    auto message = std_msgs::msg::Bool();
    message.data = true; //"Hello multi world! " + std::to_string(count_++);
    for (auto & pub : publishers_) {
      pub->publish(message);
    }

    // RCLCPP_INFO(this->get_logger(), "Nachricht gesendet über %zu Publisher: %s",
    //             publishers_.size(), message.data.c_str());
  }

  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  int publisher_index_;
  const int max_publishers_ = 6;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
