#include <fstream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cerrno>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

class Listener : public rclcpp::Node
{
public:
    Listener()
    : Node("listener")
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

        subscription_ = this->create_subscription<std_msgs::msg::UInt64>(
            topic_name_, 10,
            std::bind(&Listener::topic_callback, this, std::placeholders::_1));

        log_file_ = log_base_path_ + "/received.txt";
    }

private:
    void topic_callback(const std_msgs::msg::UInt64::SharedPtr msg)
    {
        auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        std::ofstream out(log_file_, std::ios_base::app);
        if (out.is_open()) {
            out << msg->data << " " << now << "\n";
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to open log file for writing: %s", log_file_.c_str());
        }
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

    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
    std::string topic_name_;
    double frequency_;
    std::string log_base_path_;
    std::string log_file_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}
