#include <rclcpp/rclcpp.hpp>
#include <random>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
using namespace std::chrono_literals;

class TcpNode : public rclcpp::Node {
public:
  TcpNode() : Node("tcp_node") {
    timer_ = create_wall_timer(500ms, [this]{ tick(); });
  }
private:
  int sock_ = -1;
  std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<int> dist{0,255};
  rclcpp::TimerBase::SharedPtr timer_;

  void ensure_conn(){
    if(sock_>=0) return;
    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{}; a.sin_family=AF_INET; a.sin_port=htons(5005);
    inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
    if(connect(sock_, (sockaddr*)&a, sizeof(a))<0){
      RCLCPP_WARN(get_logger(), "connect fail, will retry");
      close(sock_); sock_ = -1;
    }
  }
  void tick(){
    ensure_conn(); if(sock_<0) return;
    uint8_t v = static_cast<uint8_t>(dist(rng));
    if(send(sock_, &v, 1, 0)!=1){ RCLCPP_WARN(get_logger(),"send fail"); close(sock_); sock_=-1; return; }
    RCLCPP_INFO(get_logger(), "sent TCP %u", v);
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpNode>());
  rclcpp::shutdown();
  return 0;
}
