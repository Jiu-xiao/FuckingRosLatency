#include <cstdint>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class MultiImageSubscriberNode : public rclcpp::Node
{
public:
  MultiImageSubscriberNode()
  : rclcpp::Node("image_subscriber_node"),
    count_(0),
    latency_sum_ms_(0.0)
  {
    auto qos = rclcpp::SensorDataQoS();

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "test_image",
      qos,
      std::bind(&MultiImageSubscriberNode::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[multi_sub] listening on test_image");
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    auto now = this->now();
    auto dt = now - msg->header.stamp;
    double latency_ms = dt.seconds() * 1000.0;

    ++count_;
    latency_sum_ms_ += latency_ms;

    if (count_ % 100 == 0) {
      double avg = latency_sum_ms_ / static_cast<double>(count_);
      RCLCPP_INFO(
        this->get_logger(),
        "[multi_sub] frames=%llu avg=%.3f ms last=%.3f ms",
        static_cast<unsigned long long>(count_),
        avg,
        latency_ms);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::uint64_t count_;
  double latency_sum_ms_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiImageSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
