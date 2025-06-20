#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

class HelloPublisher : public rclcpp::Node {
public:
    HelloPublisher()
    : Node("hello_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("hello_topic", 10);
        timer_ = this->create_wall_timer(
            10ms, std::bind(&HelloPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello World " + std::to_string(count_);

        auto now = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
        publisher_->publish(message);

        std::string fileName = "send_" + getpid() + std::string(".txt");
        std::ofstream out(fileName.c_str(), std::ios_base::app);
        out << count_ << " " << now << "\n";

        count_++;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloPublisher>());
    rclcpp::shutdown();
    return 0;
}
