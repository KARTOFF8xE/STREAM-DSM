#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cerrno>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
    Talker()
    : Node("talker"), count_(0)
    {
        this->declare_parameter<std::string>("topic", "default_topic");
        this->declare_parameter<double>("frequency", 1.0);
        this->declare_parameter<std::string>("log_base_path", "");

        this->get_parameter("topic", topic_name_);
        this->get_parameter("frequency", frequency_);
        this->get_parameter("log_base_path", log_base_path_);

        if (!log_base_path_.empty()) {
            int ret = mkdir_recursive(log_base_path_);
            if (ret != 0) {
                RCLCPP_WARN(this->get_logger(), "Could not create log directory: %s", log_base_path_.c_str());
            }
        }

        publisher_ = this->create_publisher<std_msgs::msg::UInt64>(topic_name_, 10);

        if (frequency_ > 0.0) {
            auto interval = std::chrono::duration<double>(1.0 / frequency_);
            timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(interval),
                std::bind(&Talker::timer_callback, this));
        } else {
            RCLCPP_INFO(this->get_logger(), "Frequency is zero or negative, no messages will be sent.");
        }

        log_file_ = log_base_path_ + "/send.txt";
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::UInt64();
        message.data = count_;

        publisher_->publish(message);

        auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        std::ofstream out(log_file_, std::ios_base::app);
        if (out.is_open()) {
            out << count_ << " " << now << "\n";
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to open log file for writing: %s", log_file_.c_str());
        }

        count_++;
    }

    int mkdir_recursive(const std::string &path)
    {
        size_t pos = 0;
        int ret = 0;

        do {
            pos = path.find('/', pos + 1);
            std::string subdir = path.substr(0, pos);

            if (subdir.length() > 0) {
                ret = mkdir(subdir.c_str(), 0755);
                if (ret != 0 && errno != EEXIST) {
                    return ret;
                }
            }
        } while (pos != std::string::npos);

        return 0;
    }

    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string topic_name_;
    double frequency_;
    uint64_t count_;

    std::string log_base_path_;
    std::string log_file_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
