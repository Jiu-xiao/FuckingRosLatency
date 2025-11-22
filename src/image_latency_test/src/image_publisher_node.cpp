#include <chrono>
#include <vector>
#include <cstdint>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class MultiImagePublisherNode : public rclcpp::Node
{
public:
  MultiImagePublisherNode()
  : rclcpp::Node("image_publisher_node"),
    width_(1440),
    height_(1080),
    publish_rate_(30.0)
  {
    this->declare_parameter<int>("width", width_);
    this->declare_parameter<int>("height", height_);
    this->declare_parameter<double>("publish_rate", publish_rate_);

    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
    this->get_parameter("publish_rate", publish_rate_);

    auto qos = rclcpp::SensorDataQoS();
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("test_image", qos);

    image_data_.resize(
      static_cast<std::size_t>(width_) *
      static_cast<std::size_t>(height_) * 3u);

    for (std::size_t i = 0; i < image_data_.size(); ++i) {
      image_data_[i] = static_cast<std::uint8_t>(i % 256);
    }

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));

    timer_ = this->create_wall_timer(
      period,
      std::bind(&MultiImagePublisherNode::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "[multi_pub] width=%d height=%d rate=%.2f Hz",
      width_, height_, publish_rate_);
  }

private:
  void on_timer()
  {
    sensor_msgs::msg::Image msg;
    msg.header.frame_id = "camera";
    msg.height = static_cast<std::uint32_t>(height_);
    msg.width  = static_cast<std::uint32_t>(width_);
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = msg.width * 3;
    msg.data = image_data_;
    msg.header.stamp = this->now();
    publisher_->publish(std::move(msg));
  }

  int width_;
  int height_;
  double publish_rate_;

  std::vector<std::uint8_t> image_data_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiImagePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
