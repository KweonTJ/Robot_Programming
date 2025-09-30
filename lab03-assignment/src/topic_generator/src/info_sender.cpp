#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
using namespace std::chrono_literals;

class InfoSender : public rclcpp::Node {
public:
  InfoSender() : Node("info_sender") {
    pub_ = create_publisher<std_msgs::msg::String>("data_receiver/msg", 10);
    timer_ = create_wall_timer(1s, [this] {
      std_msgs::msg::String m;
      m.data = "2023042028 권택주";
      RCLCPP_INFO(get_logger(), "%s", m.data.c_str());
      pub_->publish(m);
    });
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InfoSender>());
  rclcpp::shutdown();
  return 0;
}
