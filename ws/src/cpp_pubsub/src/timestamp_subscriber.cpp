#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

class HelloSubscriber : public rclcpp::Node {
public:
    HelloSubscriber()
    : Node("hello_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "hello_topic", 10,
            std::bind(&HelloSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        auto now = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();

        std::string data = msg->data;
        std::istringstream iss(data);
        std::string word;
        int msg_num = -1;

        while (iss >> word) {
            try {
                msg_num = std::stoi(word);
            } catch (...) {
                continue;
            }
        }

        if (msg_num >= 0) {
            std::ofstream out("received.txt", std::ios_base::app);
            out << msg_num << " " << now << "\n";
        }

        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloSubscriber>());
    rclcpp::shutdown();
    return 0;
}
