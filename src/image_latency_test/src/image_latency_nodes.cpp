#include "image_latency_test/image_latency_nodes.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

ImagePublisherNode::ImagePublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_publisher_node", options),
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
    image_data_[i] = static_cast<uint8_t>(i % 256);
  }

  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));

  timer_ = this->create_wall_timer(
    period,
    std::bind(&ImagePublisherNode::timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "ImagePublisherNode: width=%d height=%d rate=%.2f Hz",
    width_, height_, publish_rate_);
}

void ImagePublisherNode::timer_callback()
{
  sensor_msgs::msg::Image msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "camera";
  msg.height = static_cast<uint32_t>(height_);
  msg.width = static_cast<uint32_t>(width_);
  msg.encoding = "rgb8";
  msg.is_bigendian = false;
  msg.step = msg.width * 3;
  msg.data = image_data_;

  publisher_->publish(std::move(msg));
}

// --------------------------------------------------------------------

ImageSubscriberNode::ImageSubscriberNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_subscriber_node", options),
  count_(0),
  latency_sum_ms_(0.0)
{
  auto qos = rclcpp::SensorDataQoS();

  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "test_image",
    qos,
    std::bind(&ImageSubscriberNode::image_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "ImageSubscriberNode started, listening on test_image.");
}

void ImageSubscriberNode::image_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  auto now = this->now();
  auto dt = now - msg->header.stamp;
  double latency_ms = dt.seconds() * 1000.0;

  ++count_;
  latency_sum_ms_ += latency_ms;

  RCLCPP_INFO(
    this->get_logger(),
    "Frame %llu latency: %.3f ms",
    static_cast<unsigned long long>(count_),
    latency_ms);

  if (count_ % 100 == 0) {
    double avg = latency_sum_ms_ / static_cast<double>(count_);
    RCLCPP_INFO(
      this->get_logger(),
      "Average latency over %llu frames: %.3f ms",
      static_cast<unsigned long long>(count_),
      avg);
  }
}
