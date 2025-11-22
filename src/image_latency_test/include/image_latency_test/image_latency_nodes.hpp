#ifndef IMAGE_LATENCY_NODES_HPP_
#define IMAGE_LATENCY_NODES_HPP_

#include <cstdint>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImagePublisherNode : public rclcpp::Node
{
public:
  explicit ImagePublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int width_;
  int height_;
  double publish_rate_;  // Hz

  std::vector<uint8_t> image_data_;
};

class ImageSubscriberNode : public rclcpp::Node
{
public:
  explicit ImageSubscriberNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  std::uint64_t count_;
  double latency_sum_ms_;
};

#endif  // IMAGE_LATENCY_NODES_HPP_
