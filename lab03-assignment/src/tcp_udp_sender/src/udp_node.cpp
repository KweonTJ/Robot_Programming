#include <rclcpp/rclcpp.hpp>
#include <random>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
using namespace std::chrono_literals;

class UdpNode : public rclcpp::Node {
public:
  UdpNode() : Node("udp_node") {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    addr_.sin_family = AF_INET; addr_.sin_port = htons(5006);
    inet_pton(AF_INET, "127.0.0.1", &addr_.sin_addr);
    timer_ = create_wall_timer(1s, [this]{ tick(); });
  }
  ~UdpNode(){ if(sock_>=0) close(sock_); }
private:
  int sock_;
  sockaddr_in addr_{};
  std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<int> dist{0,255};
  rclcpp::TimerBase::SharedPtr timer_;

  void tick(){
    uint8_t v = static_cast<uint8_t>(dist(rng));
    sendto(sock_, &v, 1, 0, (sockaddr*)&addr_, sizeof(addr_));
    RCLCPP_INFO(get_logger(), "sent UDP %u", v);
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpNode>());
  rclcpp::shutdown();
  return 0;
}
