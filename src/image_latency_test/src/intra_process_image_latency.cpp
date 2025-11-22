#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

using Image = sensor_msgs::msg::Image;

class IntraImagePipelineNode : public rclcpp::Node {
public:
  explicit IntraImagePipelineNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("intra_process_image_latency", options), width_(1440),
        height_(1080), publish_rate_(30.0), count_(0), latency_sum_ms_(0.0) {
    // 参数
    this->declare_parameter<int>("width", width_);
    this->declare_parameter<int>("height", height_);
    this->declare_parameter<double>("publish_rate", publish_rate_);

    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
    this->get_parameter("publish_rate", publish_rate_);

    // 预分配图像数据
    image_data_.resize(static_cast<std::size_t>(width_) *
                       static_cast<std::size_t>(height_) * 3u);

    for (std::size_t i = 0; i < image_data_.size(); ++i) {
      image_data_[i] = static_cast<std::uint8_t>(i % 256);
    }

    // QoS：和多进程保持一致，用 SensorDataQoS
    auto qos = rclcpp::SensorDataQoS();

    // 进程内通信：显式开启 intra-process
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> pub_options;
    pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    publisher_ =
        this->create_publisher<Image>("test_image_intra", qos, pub_options);

    // 订阅端：使用 UniquePtr 回调，配合 intra-process 零拷贝消息对象
    subscription_ = this->create_subscription<Image>(
        "test_image_intra", qos,
        std::bind(&IntraImagePipelineNode::on_image, this,
                  std::placeholders::_1));

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_));

    timer_ = this->create_wall_timer(
        period, std::bind(&IntraImagePipelineNode::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "[intra] width=%d height=%d rate=%.2f Hz",
                width_, height_, publish_rate_);
  }

private:
  void on_timer() {
    // 关键：用 unique_ptr 发布，配合 intra-process manager 走指针路径
    auto msg = std::make_unique<Image>();

    msg->header.frame_id = "camera";
    msg->height = static_cast<std::uint32_t>(height_);
    msg->width = static_cast<std::uint32_t>(width_);
    msg->encoding = "rgb8";
    msg->is_bigendian = false;
    msg->step = msg->width * 3;
    msg->data = image_data_;
    msg->header.stamp = this->now();

    publisher_->publish(std::move(msg));
  }

  // 使用 UniquePtr 回调：pub -> intra-process manager -> UniquePtr 直接交给 sub
  void on_image(Image::UniquePtr msg) {
    auto now = this->now();
    auto dt = now - msg->header.stamp;
    double latency_ms = dt.seconds() * 1000.0;

    ++count_;
    latency_sum_ms_ += latency_ms;

    if (count_ % 100 == 0) {
      double avg = latency_sum_ms_ / static_cast<double>(count_);
      RCLCPP_INFO(this->get_logger(),
                  "[intra] frames=%llu avg=%.3f ms last=%.3f ms",
                  static_cast<unsigned long long>(count_), avg, latency_ms);
    }
  }

  int width_;
  int height_;
  double publish_rate_;
  std::vector<std::uint8_t> image_data_;

  rclcpp::Publisher<Image>::SharedPtr publisher_;
  rclcpp::Subscription<Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::uint64_t count_;
  double latency_sum_ms_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true); // 开启 intra-process

  auto node = std::make_shared<IntraImagePipelineNode>(options);

  // 单线程 executor，减少调度抖动
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
