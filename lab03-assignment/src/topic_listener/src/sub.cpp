#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <optional>

class Sub : public rclcpp::Node {
public:
  Sub() : Node("sub") {
    sub_info_ = create_subscription<std_msgs::msg::String>(
      "data_receiver/msg", 10, [this](std_msgs::msg::String::SharedPtr m){
        info_ = m->data;
        if(!num_) RCLCPP_WARN(get_logger(), "number not received yet");
        else RCLCPP_INFO(get_logger(), "%s %u", info_->c_str(), *num_);
      });
    sub_num_ = create_subscription<std_msgs::msg::UInt8>(
      "topic_generator/msg", 10, [this](std_msgs::msg::UInt8::SharedPtr m){
        num_ = m->data;
        if(info_) RCLCPP_INFO(get_logger(), "%s %u", info_->c_str(), *num_);
      });
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_info_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_num_;
  std::optional<std::string> info_;
  std::optional<uint8_t> num_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sub>());
  rclcpp::shutdown();
  return 0;
}
