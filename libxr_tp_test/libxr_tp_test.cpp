#include "libxr.hpp"
#include "libxr_def.hpp"
#include "message.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <thread>

using namespace LibXR;

// ========== 通用配置 ==========

static constexpr uint32_t NUM_FRAMES = 300;     // 每种分辨率发多少帧
static constexpr double PUBLISH_RATE_HZ = 30.0; // 模拟 ROS 30Hz

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Micro = std::chrono::duration<double, std::micro>;

// 简单统计结构
struct LatencyStats {
  double min_us = std::numeric_limits<double>::max();
  double max_us = 0.0;
  double sum_us = 0.0;
  uint32_t count = 0;

  void add(double us) {
    if (us < min_us)
      min_us = us;
    if (us > max_us)
      max_us = us;
    sum_us += us;
    ++count;
  }

  void log(const char *name) const {
    if (count == 0) {
      XR_LOG_WARN("%s: no samples", name);
      return;
    }
    double avg = sum_us / static_cast<double>(count);
    // 用 PASS 保证一定能看到结果
    XR_LOG_PASS("[RESULT] %s: count=%u avg=%.3f us min=%.3f us max=%.3f us",
                name, count, avg, min_us, max_us);
  }
};

// 发布时间戳（按 seq 索引）
static std::array<TimePoint, NUM_FRAMES> g_pub_time{};

// 回调 & 同步统计
static LatencyStats g_cb_entry_stats; // 发布 -> 回调入口
static LatencyStats g_cb_copy_stats;  // 回调里的额外拷贝
static LatencyStats g_sync_stats;     // 发布 -> Sync Wait OK

static void ResetStats() {
  g_pub_time.fill(TimePoint{});
  g_cb_entry_stats = LatencyStats{};
  g_cb_copy_stats = LatencyStats{};
  g_sync_stats = LatencyStats{};
}

// ========== 两种图像类型 ==========

template <uint32_t W, uint32_t H> struct ImageFrameT {
  static constexpr uint32_t WIDTH = W;
  static constexpr uint32_t HEIGHT = H;

  uint32_t width;
  uint32_t height;
  uint32_t seq;            // 帧号
  uint8_t data[W * H * 3]; // RGB888
};

using ImageFrame1440 = ImageFrameT<1440, 1080>;
using ImageFrame320 = ImageFrameT<320, 240>;

// ========== 单一分辨率测试函数（堆上分配 Frame） ==========

template <typename Frame>
void RunImageTest(const char *label, const char *topic_name,
                  LibXR::Topic::Domain &domain) {
  ResetStats();

  XR_LOG_INFO("===== Test %s (%ux%u) BEGIN =====", label, Frame::WIDTH,
              Frame::HEIGHT);

  // 1. 创建 Topic：cache=false, check_length=false
  auto topic = LibXR::Topic::CreateTopic<Frame>(topic_name, &domain,
                                                /*multi_publisher=*/false,
                                                /*cache=*/false,
                                                /*check_length=*/false);

  // 2. 同步订阅（独立线程 Wait），数据缓冲放在堆上
  auto sync_frame = std::make_unique<Frame>();
  LibXR::Topic::SyncSubscriber<Frame> sync_sub(topic, *sync_frame);

  // 3. 回调订阅，回调缓冲放在堆上
  auto cb_frame = std::make_unique<Frame>();

  auto cb = LibXR::Topic::Callback::Create(
      [](bool /*in_isr*/, Frame *dst, LibXR::RawData &raw) {
        auto now = Clock::now();
        auto *src = reinterpret_cast<Frame *>(raw.addr_);
        uint32_t seq = src->seq;

        // ① 发布 -> 回调入口 延迟
        if (seq < NUM_FRAMES) {
          TimePoint pub_tp = g_pub_time[seq];
          if (pub_tp.time_since_epoch().count() != 0) {
            double entry_us =
                std::chrono::duration_cast<Micro>(now - pub_tp).count();
            g_cb_entry_stats.add(entry_us);
            XR_LOG_DEBUG("[CB][%ux%u] seq=%u entry=%.3f us", Frame::WIDTH,
                         Frame::HEIGHT, seq, entry_us);
          }
        }

        // ② 回调里：先正常拷贝一次，再额外拷贝一次，测“额外 copy”的耗时
        *dst = *src; // 正常 copy（比如业务逻辑）
        auto t2 = Clock::now();
        *dst = *src; // 额外 copy，用来测 copy 开销
        auto t3 = Clock::now();

        double extra_copy_us =
            std::chrono::duration_cast<Micro>(t3 - t2).count();
        g_cb_copy_stats.add(extra_copy_us);

        XR_LOG_DEBUG("[CB][%ux%u] seq=%u extra_copy=%.3f us", Frame::WIDTH,
                     Frame::HEIGHT, seq, extra_copy_us);
      },
      cb_frame.get());

  topic.RegisterCallback(cb);

  XR_LOG_PASS("Topic & subscribers for %s (%ux%u) set up", label, Frame::WIDTH,
              Frame::HEIGHT);

  // 4. 同步订阅线程
  std::thread sync_thread([&]() {
    for (uint32_t i = 0; i < NUM_FRAMES; ++i) {
      auto ec = sync_sub.Wait(5000); // 防止挂死：5s 超时
      auto now = Clock::now();

      if (ec == ErrorCode::OK) {
        uint32_t seq = sync_frame->seq;
        if (seq < NUM_FRAMES) {
          TimePoint pub_tp = g_pub_time[seq];
          if (pub_tp.time_since_epoch().count() != 0) {
            double sync_us =
                std::chrono::duration_cast<Micro>(now - pub_tp).count();
            g_sync_stats.add(sync_us);
            XR_LOG_DEBUG("[SYNC][%ux%u] seq=%u latency=%.3f us", Frame::WIDTH,
                         Frame::HEIGHT, seq, sync_us);
          } else {
            XR_LOG_WARN("[SYNC][%ux%u] seq=%u has no pub timestamp",
                        Frame::WIDTH, Frame::HEIGHT, seq);
          }
        } else {
          XR_LOG_WARN("[SYNC][%ux%u] seq=%u out of range", Frame::WIDTH,
                      Frame::HEIGHT, seq);
        }
      } else {
        XR_LOG_ERROR("[SYNC][%ux%u] Wait failed/timeout, ec=%d", Frame::WIDTH,
                     Frame::HEIGHT, static_cast<int>(ec));
      }
    }

    XR_LOG_PASS("Sync subscriber thread for %s finished", label);
  });

  // 5. 发布循环（main 线程），发布缓冲也在堆上
  const double period_sec = 1.0 / PUBLISH_RATE_HZ;

  auto frame = std::make_unique<Frame>();
  frame->width = Frame::WIDTH;
  frame->height = Frame::HEIGHT;

  for (uint32_t seq = 0; seq < NUM_FRAMES; ++seq) {
    frame->seq = seq;

    // 记录发布时间（给 entry/sync 统计用）
    g_pub_time[seq] = Clock::now();

    // 填充一点数据，模拟图像负载
    std::memset(frame->data, static_cast<int>(seq & 0xFF), sizeof(frame->data));

    topic.Publish(*frame);

    std::this_thread::sleep_for(std::chrono::duration<double>(period_sec));
  }

  XR_LOG_PASS("Publisher for %s sent %u frames", label, NUM_FRAMES);

  if (sync_thread.joinable()) {
    sync_thread.join();
  }

  // 6. 打印统计结果（用 PASS，肯定能看到）
  XR_LOG_PASS("===== Stats for %s (%ux%u) =====", label, Frame::WIDTH,
              Frame::HEIGHT);
  g_cb_entry_stats.log("Callback entry latency (Publish -> CB)");
  g_cb_copy_stats.log("Callback extra copy latency (2nd copy in CB)");
  g_sync_stats.log("Sync subscriber latency (Publish -> Wait OK)");
  XR_LOG_PASS("===== Test %s (%ux%u) END =====", label, Frame::WIDTH,
              Frame::HEIGHT);
}

// ========== main：依次测试 1440×1080 和 320×240 ==========

int main() {
  LibXR::PlatformInit();
  XR_LOG_PASS("Platform initialized");

  LibXR::Topic::Domain domain("image_domain");

  // 大图 1440x1080
  RunImageTest<ImageFrame1440>("image_1440x1080", "camera/image_1440", domain);

  // 小图 320x240
  RunImageTest<ImageFrame320>("image_320x240", "camera/image_320", domain);

  XR_LOG_PASS("All image latency tests finished");
  return 0;
}
