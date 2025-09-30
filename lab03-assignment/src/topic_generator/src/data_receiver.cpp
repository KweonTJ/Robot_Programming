#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <optional>
#include <thread>
#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
using namespace std::chrono_literals;

class DataReceiver : public rclcpp::Node {
public:
  DataReceiver() : Node("data_receiver") {
    pub_ = create_publisher<std_msgs::msg::UInt8>("topic_generator/msg", 10);
    tcp_th_ = std::thread([this]{ tcp_loop(); });
    udp_th_ = std::thread([this]{ udp_loop(); });
    timer_ = create_wall_timer(300ms, [this]{ compare_and_publish(); });
  }
  ~DataReceiver(){
    running_ = false;
    if(tcp_th_.joinable()) tcp_th_.join();
    if(udp_th_.joinable()) udp_th_.join();
  }
private:
  // latest samples
  std::mutex mtx_;
  std::optional<uint8_t> tcp_v_, udp_v_, last_winner_;
  bool running_ = true;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread tcp_th_, udp_th_;

  void tcp_loop(){
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt=1; setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in a{}; a.sin_family=AF_INET; a.sin_port=htons(5005); a.sin_addr.s_addr=INADDR_ANY;
    bind(srv,(sockaddr*)&a,sizeof(a)); listen(srv,1);
    while(running_){
      int cli = accept(srv, nullptr, nullptr);
      if(cli<0) continue;
      while(running_){
        uint8_t b; ssize_t n = recv(cli, &b, 1, 0);
        if(n<=0) break;
        std::scoped_lock lk(mtx_); tcp_v_ = b;
      }
      close(cli);
    }
    close(srv);
  }
  void udp_loop(){
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in a{}; a.sin_family=AF_INET; a.sin_port=htons(5006); a.sin_addr.s_addr=INADDR_ANY;
    bind(s,(sockaddr*)&a,sizeof(a));
    while(running_){
      uint8_t b; ssize_t n = recv(s, &b, 1, 0);
      if(n==1){ std::scoped_lock lk(mtx_); udp_v_ = b; }
    }
    close(s);
  }
  void publish(uint8_t v){ std_msgs::msg::UInt8 m; m.data = v; pub_->publish(m); }
  void compare_and_publish(){
    std::optional<uint8_t> t,u; { std::scoped_lock lk(mtx_); t=tcp_v_; u=udp_v_; }
    if(!t || !u) return; // 초기엔 둘 다 받을 때까지 대기
    if(*u > *t){
      RCLCPP_INFO(get_logger(), "UDP was larger. UDP: %u, TCP: %u", *u, *t);
      last_winner_ = *u; publish(*last_winner_);
    } else if(*t > *u){
      RCLCPP_INFO(get_logger(), "TCP was larger. TCP: %u, UDP: %u", *t, *u);
      last_winner_ = *t; publish(*last_winner_);
    } else {
      RCLCPP_INFO(get_logger(), "Values are equal. Both: %u", *t);
      if(last_winner_) publish(*last_winner_); else publish(0);
    }
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataReceiver>());
  rclcpp::shutdown();
  return 0;
}
