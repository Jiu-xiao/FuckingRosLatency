# FuckingRosLatency

对比 ROS 2 图像传输链路与 LibXR::Topic 自定义消息系统在 **延迟** 和 **CPU 占用** 上的表现。  
所有测试均以 RGB 图像帧为载荷，发布频率 30 Hz，编译使用 `-O3`。

---

## 仓库结构

假定工作空间根目录为 `ros2_ws`：

```text
ros2_ws/
├── auto_bench_image_latency.sh        # ROS 2 图像延迟基准脚本
├── auto_bench_libxr_image_latency.sh  # LibXR 图像延迟基准脚本
├── build/                             # colcon 构建输出
├── install/                           # colcon 安装空间
├── src/
│   └── image_latency_test/            # ROS 2 测试包
└── libxr_tp_test/
    ├── CMakeLists.txt                 # LibXR 测试包
    ├── libxr/                         # LibXR 源码（上游仓库克隆）
    └── libxr_tp_test.cpp              # LibXR 图像基准程序
```

---

## 环境与构建

### 依赖

参考环境（CI）：

- Ubuntu 22.04
- ROS 2 Humble（`ros-humble-ros-base`）
- `python3-colcon-common-extensions`
- `build-essential`, `cmake`
- `sysstat`（用于 `pidstat`）
- `pkg-config`
- `libudev-dev`, `libwpa-client-dev`, `libnm-dev`（LibXR 依赖）

安装示例（简化）：

```bash
sudo apt-get update
sudo apt-get install -y curl gnupg2 lsb-release

# ROS 2 源
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros-base \
  python3-colcon-common-extensions \
  build-essential \
  cmake \
  sysstat \
  pkg-config \
  libudev-dev \
  libwpa-client-dev \
  libnm-dev
```

LibXR 源码需克隆到 `libxr_tp_test/libxr`：

```bash
cd ros2_ws/libxr_tp_test
git clone --depth 1 https://github.com/Jiu-xiao/libxr.git libxr
```

### 构建

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash

colcon build
# 或：
# colcon build --packages-select image_latency_test libxr_tp_test
```

编译完成后：

- ROS 2 节点：`install/image_latency_test/lib/image_latency_test/`
- LibXR 基准：`build/libxr_tp_test/libxr_tp_test`

---

## 运行基准

脚本统一使用环境变量：

- `WS`：工作空间路径（默认 `$HOME/ros2_ws`）
- `DURATION`：每种模式运行时长（秒，默认 30）
- `RATE` / `WIDTH` / `HEIGHT`：发布帧率和分辨率（只影响 ROS 2 测试）

典型调用：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
export WS=$(pwd)

# ROS 2：1440x1080 @ 30 Hz
./auto_bench_image_latency.sh

# ROS 2：320x240 @ 30 Hz
WIDTH=320 HEIGHT=240 ./auto_bench_image_latency.sh

# LibXR：内部依次测试 1440x1080 / 320x240
./auto_bench_libxr_image_latency.sh
```

所有日志输出到 `ros2_ws/logs/`，脚本会在控制台打印 `[RESULT]` 汇总行。

---

## 实现概览

### ROS 2

包 `image_latency_test`：

- `image_publisher_node`
  - 发布 `sensor_msgs/msg/Image`（`rgb8`），话题 `test_image`。
  - 启动时预生成一块只读图像缓冲区（对应目标分辨率），后续周期复用。
  - 在定时器回调中：
    1. 构造 `sensor_msgs::msg::Image`，从预生成缓冲区拷贝一帧到 `msg.data`；
    2. **完成拷贝后**，执行 `msg.header.stamp = now()`；
    3. 紧接着调用 `publisher_->publish(std::move(msg))`。
  - 因此订阅端计算的 `now() - msg.header.stamp` 只覆盖：
    - ROS 2 通信路径（多进程 / intra-process）和调度开销，
    - **不包含** 从预生成缓冲到消息的数据拷贝时间。

- `image_subscriber_node`
  - 订阅 `test_image`。
  - 回调中读取 `msg.header.stamp`，计算 `now() - msg.header.stamp`，周期性打印平均延迟。
  - 该延迟仅反映 **消息从发布端打时间戳到订阅回调触发** 的时间。

- `intra_process_image_latency`
  - 发布 / 订阅在同一进程中。
  - 显式启用 intra-process 通信。
  - 发布端使用 `std::unique_ptr<Image>`，订阅端使用 `Image::UniquePtr`。
  - 时间戳打点顺序与多进程版本保持一致：**先拷贝，后 stamp，再 publish**，因此测得的是 intra-process 路径本身的开销，同样不含帧拷贝时间。

脚本 `auto_bench_image_latency.sh`：

- 按“多进程 pub/sub → intra-process 单进程”顺序运行。
- 用 `pidstat -u 1 -p <PID>` 分别记录 pub、sub、intra 三个进程/实例的 CPU 占用。

---

### LibXR

程序 `libxr_tp_test.cpp`（单一可执行），对两个分辨率依次执行：

- 为每种帧类型（1440×1080 / 320×240）创建 `Topic<Frame>`。
- 注册两个订阅者：
  - 一个同步订阅者（独立线程调用 `Wait()`）；
  - 一个回调订阅者（收到帧立刻执行回调函数）。

- 主线程以 30 Hz 发送 `NUM_FRAMES = 300` 帧，每帧流程为：
  1. 填充对应分辨率的 RGB888 数据到帧缓冲；
  2. **完成帧填充后**，记录 `g_pub_time[seq] = now()`；
  3. 调用 `topic.Publish(frame)`。

  因此，所有用 `g_pub_time[seq]` 作为起点的延迟统计，**不包含帧填充时间**。

- 回调订阅者中：
  1. 根据 `seq` 查 `g_pub_time[seq]`，计算 **发布 → 回调入口** 延迟；
  2. 再进行一次完整帧拷贝到业务缓冲区（模拟真实处理逻辑）；
  3. 额外执行两次完整帧拷贝并分别计时：
     - 一次使用 `std::memcpy`；
     - 一次使用 `LibXR::Memory::FastCopy`。
  4. 这些 memcpy/FastCopy 的计时仅反映 **业务侧额外拷贝** 的成本，与“发布 → 回调入口”的延迟解耦。

- 同步订阅线程：
  - 独立线程中循环调用 `topic.Wait(frame)`。
  - 当 `Wait()` 返回时，根据帧内的 `seq` 和 `g_pub_time[seq]` 计算 **发布 → Wait 成功** 的延迟。
  - LibXR 的同步订阅在内部会将消息拷贝 / 迁移到订阅线程可见的缓冲区，以保证跨线程安全；

脚本 `auto_bench_libxr_image_latency.sh`：

- 启动基准程序的同时，用 `pidstat` 记录整个进程 CPU 占用。
- 从程序日志中抽取 `[RESULT]` 行，并汇总延迟统计和 CPU 占用。

---

## 测试配置

所有结果均来自 GitHub Actions 上[最近一次运行](https://github.com/Jiu-xiao/FuckingRosLatency/actions/runs/19598484409/job/56126494824)，配置如下：

- OS：Ubuntu 22.04（GitHub 托管 runner）
- ROS：Humble
- 帧率：30 Hz
- 每种模式运行时长：30 s
- 每种分辨率测试帧数：
  - ROS 2：约 900 帧（30 Hz × 30 s）
  - LibXR：固定 300 帧 / 分辨率
- 分辨率：
  - 1440×1080
  - 320×240

---

## 结果与对比

### 1440×1080 分辨率

#### ROS 2

单位：延迟为毫秒（ms），CPU 为百分比。

| 模式                | 指标                          | 数值                            |
|---------------------|-------------------------------|---------------------------------|
| 多进程 pub/sub      | sub 延迟（900 帧）           | avg = **1.779 ms** (min ≈ 1.551, max ≈ 2.021) |
|                     | pub CPU（29 样本）           | avg = **4.76 %**                |
|                     | sub CPU（29 样本）           | avg = **3.45 %**                |
| intra-process 单进程 | 延迟（900 帧）               | avg = **0.026 ms** (min ≈ 0.019, max ≈ 0.036) |
|                     | CPU（29 样本）               | avg = **1.38 %**                |

观察：

- 对大图像帧，多进程模式的链路包含序列化 / 反序列化、进程间通信和调度开销，平均延迟约 1.8 ms。
- 启用 intra-process 后，延迟下降到 ~26 µs 量级，CPU 也明显降低。

#### LibXR

单位：延迟为微秒（µs），CPU 为百分比。

| 指标                                                     | 数值                                      |
|----------------------------------------------------------|-------------------------------------------|
| Callback entry latency (Publish -> CB)                   | count=300，avg = **0.549 µs** (min=0.181, max=2.144) |
| Callback memcpy latency (std::memcpy Frame)              | count=300，avg = **299.693 µs** (min=206.427, max=1528.852) |
| Callback FastCopy latency (LibXR::Memory::FastCopy Frame)| count=300，avg = **249.836 µs** (min=201.667, max=832.058) |
| Sync subscriber latency (Publish -> Wait OK)             | count=300，avg = **1063.704 µs** (min=929.126, max=2746.869) |
| 进程 CPU（libxr_tp_test）                                | samples=19，avg = **2.00 %**              |

关键点：

- 发布 → 回调入口延迟在 **亚微秒级**，说明 LibXR 的消息分发本身开销极低。
- 回调中对整帧的多次拷贝（业务拷贝 + 额外 `memcpy` + 额外 `FastCopy`）是主要时间和 CPU 成本。
- 发布 → 同步订阅线程 `Wait` 返回的延迟在 ~1.06 ms，包含用户层同步与线程调度等因素。

---

### 320×240 分辨率

#### ROS 2

| 模式                | 指标                          | 数值                            |
|---------------------|-------------------------------|---------------------------------|
| 多进程 pub/sub      | sub 延迟（900 帧）           | avg = **0.222 ms** (min ≈ 0.184, max ≈ 0.331) |
|                     | pub CPU（29 样本）           | avg = **0.59 %**                |
|                     | sub CPU（29 样本）           | avg = **0.48 %**                |
| intra-process 单进程 | 延迟（900 帧）               | avg = **0.024 ms** (min ≈ 0.020, max ≈ 0.046) |
|                     | CPU（29 样本）               | avg = **0.31 %**                |

与 1440×1080 对比：

- 图像更小后，多进程模式延迟和 CPU 占用都显著下降。
- intra-process 延迟在两种分辨率下接近，都在 ~25 µs 量级，说明其主要成本是一次数据拷贝，受分辨率影响不大。

#### LibXR

| 指标                                                     | 数值                                      |
|----------------------------------------------------------|-------------------------------------------|
| Callback entry latency (Publish -> CB)                   | count=300，avg = **0.447 µs** (min=0.220, max=1.262) |
| Callback memcpy latency (std::memcpy Frame)              | count=300，avg = **13.197 µs** (min=9.448, max=106.810) |
| Callback FastCopy latency (LibXR::Memory::FastCopy Frame)| count=300，avg = **13.778 µs** (min=10.289, max=109.364) |
| Sync subscriber latency (Publish -> Wait OK)             | count=300，avg = **100.117 µs** (min=61.327, max=278.458) |
| 进程 CPU（libxr_tp_test）                                | samples=19，avg = **2.00 %**              |

可以看到：

- 分辨率降低后，整帧拷贝时间下降到 ~13 µs 量级，`memcpy` 与 `FastCopy` 差异不大。
- 消息分发（发布 → 回调入口）仍然保持在 1 µs 以内。
- 同步订阅延迟降到 ~0.1 ms，明显小于大分辨率场景。

---

### 综合对比与结论

按链路模式划分，可以粗略归纳为：

1. **ROS 2 多进程**
   - 延迟：从 ~0.22 ms（320×240）到 ~1.78 ms（1440×1080），随分辨率显著变化。
   - CPU：大图像下 pub/sub 合计约 8% 左右，小图像时降至 1% 以内。
   - 适合需要进程隔离、但能接受毫秒级延迟和较高 CPU 开销的场景。

2. **ROS 2 intra-process**
   - 延迟：两种分辨率都在 ~0.02–0.03 ms。
   - CPU：320×240 时约 0.31%，1440×1080 时约 1.38%。
   - 避免了 IPC 和额外调度开销，是 ROS 2 框架下的低延迟方案。

3. **LibXR 回调路径（Publish → Callback Entry）**
   - 延迟：亚微秒级（0.4–0.5 µs），明显低于 ROS intra-process。
   - 本身几乎不受分辨率影响，主要由 LibXR 内部路径决定。

4. **LibXR 回调内整帧拷贝**
   - 当前基准中，每帧在回调内发生 **三次完整拷贝**：
     - 1 次业务缓冲拷贝；
     - 1 次 `std::memcpy` 计时；
     - 1 次 `FastCopy` 计时。
   - 因此 CPU 占用约 2%，是刻意构造的“重拷贝压力”场景，不代表实际应用的最优写法。

5. **LibXR 同步订阅路径（Publish → Wait OK）**
   - 大图像下约 1.06 ms，小图像约 0.10 ms。
   - 受用户层同步实现和线程调度影响，是“跨线程 + 同步”场景的端到端数字。

该仓库可以作为后续实验的基础，例如：

- 评估不同线程模型、同步方式对订阅延迟的影响；
- 组合成更长的处理流水线，测量端到端行为；
- 在不同 CPU / 内存拓扑下（多核、NUMA 等）验证拷贝策略。