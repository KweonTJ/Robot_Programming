#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraViewer : public rclcpp::Node
{
public:
    CameraViewer()
    : Node("camera_viewer")
    {
        // Image subscriber (raw)
        image_sub_ = image_transport::create_subscription(
            this,
            "/camera/image_raw",                                   // 입력 토픽
            std::bind(&CameraViewer::imageCallback, this, std::placeholders::_1),
            "raw"
        );

        // Publisher for flipped image
        image_pub_ = image_transport::create_publisher(
            this,
            "/camera/image_flipped"                                // 출력 토픽
        );

        RCLCPP_INFO(this->get_logger(), "camera_viewer node started.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        // Convert ROS2 Image → OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");             // BGR8로 변환
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        // Flip 180 degrees (상하, 좌우 동시에 뒤집기)
        cv::Mat flipped;
        cv::flip(cv_ptr->image, flipped, -1);

        // Convert back to ROS Image
        cv_ptr->image = flipped;
        sensor_msgs::msg::Image::SharedPtr out_msg = cv_ptr->toImageMsg();

        // Publish
        image_pub_.publish(out_msg);
    }

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraViewer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
