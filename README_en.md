# FuckingRosLatency

Compare the **latency** and **CPU usage** of the ROS 2 image transport pipeline and the LibXR::Topic custom message system.  
All tests use RGB image frames as payload, with a publish rate of 30 Hz, compiled with `-O3`.

---

## Repository Layout

Assume the workspace root is `ros2_ws`:

```text
ros2_ws/
├── auto_bench_image_latency.sh        # ROS 2 image latency benchmark script
├── auto_bench_libxr_image_latency.sh  # LibXR image latency benchmark script
├── build/                             # colcon build output
├── install/                           # colcon install space
├── src/
│   └── image_latency_test/            # ROS 2 test package
└── libxr_tp_test/
    ├── CMakeLists.txt                 # LibXR test package
    ├── libxr/                         # LibXR source (cloned from upstream repo)
    └── libxr_tp_test.cpp              # LibXR image benchmark program
```

---

## Environment & Build

### Dependencies

Reference environment (CI):

- Ubuntu 22.04
- ROS 2 Humble (`ros-humble-ros-base`)
- `python3-colcon-common-extensions`
- `build-essential`, `cmake`
- `sysstat` (for `pidstat`)
- `pkg-config`
- `libudev-dev`, `libwpa-client-dev`, `libnm-dev` (LibXR dependencies)

Example installation (simplified):

```bash
sudo apt-get update
sudo apt-get install -y curl gnupg2 lsb-release

# ROS 2 apt repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key   -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
sudo apt-get install -y   ros-humble-ros-base   python3-colcon-common-extensions   build-essential   cmake   sysstat   pkg-config   libudev-dev   libwpa-client-dev   libnm-dev
```

LibXR source must be cloned into `libxr_tp_test/libxr`:

```bash
cd ros2_ws/libxr_tp_test
git clone --depth 1 https://github.com/Jiu-xiao/libxr.git libxr
```

### Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash

colcon build
# or:
# colcon build --packages-select image_latency_test libxr_tp_test
```

After a successful build you should have:

- ROS 2 nodes: `install/image_latency_test/lib/image_latency_test/`
- LibXR benchmark: `build/libxr_tp_test/libxr_tp_test`

---

## Running the Benchmarks

Both scripts use the following environment variables:

- `WS`: workspace path (default `$HOME/ros2_ws`)
- `DURATION`: runtime for each mode in seconds (default 30)
- `RATE` / `WIDTH` / `HEIGHT`: publish rate and resolution (only affects ROS 2 tests)

Typical usage:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
export WS=$(pwd)

# ROS 2: 1440x1080 @ 30 Hz
./auto_bench_image_latency.sh

# ROS 2: 320x240 @ 30 Hz
WIDTH=320 HEIGHT=240 ./auto_bench_image_latency.sh

# LibXR: internally tests 1440x1080 / 320x240 in sequence
./auto_bench_libxr_image_latency.sh
```

All logs are written to `ros2_ws/logs/`. The scripts print `[RESULT]` summary lines to the console.

---

## Implementation Overview

### ROS 2

Package `image_latency_test`:

- `image_publisher_node`
  - Publishes `sensor_msgs/msg/Image` (`rgb8`) on topic `test_image`.
  - On startup, pre-allocates a read-only image buffer (for the target resolution) and reuses it every cycle.
  - In the timer callback:
    1. Constructs a `sensor_msgs::msg::Image` and copies one frame from the pre-generated buffer into `msg.data`;
    2. **After the copy finishes**, sets `msg.header.stamp = now()`;
    3. Immediately calls `publisher_->publish(std::move(msg))`.
  - Therefore, the `now() - msg.header.stamp` computed on the subscriber side only covers:
    - ROS 2 communication path (multi-process / intra-process) and scheduling overhead,
    - and **does not include** the data copy time from the pre-generated buffer into the message.

- `image_subscriber_node`
  - Subscribes to `test_image`.
  - In the callback, reads `msg.header.stamp`, computes `now() - msg.header.stamp`, and periodically prints the average latency.
  - This latency reflects only the time **from timestamping on the publisher side to subscriber callback execution**.

- `intra_process_image_latency`
  - Publisher and subscriber live in the same process.
  - Explicitly enables intra-process communication.
  - The publisher uses `std::unique_ptr<Image>` and the subscriber uses `Image::UniquePtr`.
  - The timestamping order is kept identical to the multi-process version: **copy first, then stamp, then publish**.  
    As a result, the measured latency is the cost of the intra-process path itself and still excludes the frame copy time.

Script `auto_bench_image_latency.sh`:

- Runs the modes in order: multi-process pub/sub → intra-process single process.
- Uses `pidstat -u 1 -p <PID>` to record CPU usage of the pub, sub, and intra processes/instances separately.

---

### LibXR

Program `libxr_tp_test.cpp` (single executable) runs benchmarks for two resolutions in sequence:

- For each frame type (1440×1080 / 320×240), it creates a `Topic<Frame>`.
- Registers two subscribers:
  - a synchronous subscriber (dedicated thread calling `Wait()`), and
  - a callback subscriber (invokes a callback as soon as a frame is received).

- The main thread sends `NUM_FRAMES = 300` frames at 30 Hz. For each frame:
  1. Fills a frame buffer with RGB888 data for the given resolution;
  2. **After the frame is filled**, records `g_pub_time[seq] = now()`;
  3. Calls `topic.Publish(frame)`.

  Therefore, all latency statistics that use `g_pub_time[seq]` as the start time **exclude the frame-fill cost**.

- In the callback subscriber:
  1. Uses `seq` to look up `g_pub_time[seq]` and computes the **publish → callback-entry** latency;
  2. Performs a full-frame copy into a business buffer (simulating real processing);
  3. Performs two additional full-frame copies and times them separately:
     - one using `std::memcpy`, and
     - one using `LibXR::Memory::FastCopy`.
  4. These memcpy/FastCopy timings reflect only the **extra business-side copy cost** and are decoupled from the publish → callback-entry latency.

- Synchronous subscriber thread:
  - In a dedicated thread, loops on `topic.Wait(frame)`.
  - When `Wait()` returns, uses the frame’s `seq` and `g_pub_time[seq]` to compute the **publish → Wait success** latency.
  - LibXR’s synchronous subscription internally copies/moves the message into a buffer visible to the subscriber thread to ensure cross-thread safety.

Script `auto_bench_libxr_image_latency.sh`:

- While the benchmark program is running, uses `pidstat` to record the process’s CPU usage.
- Extracts `[RESULT]` lines from the program logs and summarizes latency statistics and CPU usage.

---

## Test Configuration

All results come from the [latest GitHub Actions run](https://github.com/Jiu-xiao/FuckingRosLatency/actions/runs/19598484409/job/56126494824), with the following configuration:

- OS: Ubuntu 22.04 (GitHub-hosted runner)
- ROS: Humble
- Frame rate: 30 Hz
- Runtime per mode: 30 s
- Frames per resolution:
  - ROS 2: ~900 frames (30 Hz × 30 s)
  - LibXR: fixed 300 frames / resolution
- Resolutions:
  - 1440×1080
  - 320×240

---

## Results & Comparison

### Resolution 1440×1080

#### ROS 2

Units: latency in milliseconds (ms), CPU in percent.

| Mode                  | Metric                   | Value                                         |
| --------------------- | ------------------------ | --------------------------------------------- |
| Multi-process pub/sub | sub latency (900 frames) | avg = **1.779 ms** (min ≈ 1.551, max ≈ 2.021) |
|                       | pub CPU (29 samples)     | avg = **4.76 %**                              |
|                       | sub CPU (29 samples)     | avg = **3.45 %**                              |
| Intra-process single  | latency (900 frames)     | avg = **0.026 ms** (min ≈ 0.019, max ≈ 0.036) |
|                       | CPU (29 samples)         | avg = **1.38 %**                              |

Observations:

- For large image frames, the multi-process pipeline includes serialization/deserialization, IPC, and scheduling overhead, resulting in an average latency of about 1.8 ms.
- Enabling intra-process reduces latency to ~26 µs and noticeably lowers CPU usage.

#### LibXR

Units: latency in microseconds (µs), CPU in percent.

| Metric                                                    | Value                                                        |
| --------------------------------------------------------- | ------------------------------------------------------------ |
| Callback entry latency (Publish → CB)                     | count=300, avg = **0.549 µs** (min=0.181, max=2.144)         |
| Callback memcpy latency (std::memcpy Frame)               | count=300, avg = **299.693 µs** (min=206.427, max=1528.852)  |
| Callback FastCopy latency (LibXR::Memory::FastCopy Frame) | count=300, avg = **249.836 µs** (min=201.667, max=832.058)   |
| Sync subscriber latency (Publish → Wait OK)               | count=300, avg = **1063.704 µs** (min=929.126, max=2746.869) |
| Process CPU (`libxr_tp_test`)                             | samples=19, avg = **2.00 %**                                 |

Key points:

- Publish → callback-entry latency is **sub-microsecond**, showing that LibXR’s message dispatch itself has extremely low overhead.
- Multiple full-frame copies in the callback (business copy + extra `memcpy` + extra `FastCopy`) dominate the time and CPU cost.
- The publish → synchronous subscriber `Wait` latency is ~1.06 ms, including user-level synchronization and thread scheduling.

---

### Resolution 320×240

#### ROS 2

| Mode                  | Metric                   | Value                                         |
| --------------------- | ------------------------ | --------------------------------------------- |
| Multi-process pub/sub | sub latency (900 frames) | avg = **0.222 ms** (min ≈ 0.184, max ≈ 0.331) |
|                       | pub CPU (29 samples)     | avg = **0.59 %**                              |
|                       | sub CPU (29 samples)     | avg = **0.48 %**                              |
| Intra-process single  | latency (900 frames)     | avg = **0.024 ms** (min ≈ 0.020, max ≈ 0.046) |
|                       | CPU (29 samples)         | avg = **0.31 %**                              |

Compared to 1440×1080:

- With smaller images, both latency and CPU usage in multi-process mode decrease significantly.
- Intra-process latency at both resolutions is similar, around ~25 µs, indicating that its main cost is a single copy, only weakly dependent on resolution.

#### LibXR

| Metric                                                    | Value                                                     |
| --------------------------------------------------------- | --------------------------------------------------------- |
| Callback entry latency (Publish → CB)                     | count=300, avg = **0.447 µs** (min=0.220, max=1.262)      |
| Callback memcpy latency (std::memcpy Frame)               | count=300, avg = **13.197 µs** (min=9.448, max=106.810)   |
| Callback FastCopy latency (LibXR::Memory::FastCopy Frame) | count=300, avg = **13.778 µs** (min=10.289, max=109.364)  |
| Sync subscriber latency (Publish → Wait OK)               | count=300, avg = **100.117 µs** (min=61.327, max=278.458) |
| Process CPU (`libxr_tp_test`)                             | samples=19, avg = **2.00 %**                              |

We can see:

- Reducing the resolution brings the full-frame copy time down to ~13 µs. `memcpy` and `FastCopy` are close on this workload.
- Message dispatch (publish → callback-entry) remains below 1 µs.
- Synchronous subscription latency drops to ~0.1 ms, much smaller than in the high-resolution case.

---

### Overall Comparison & Conclusions

By communication mode, we can roughly summarize:

1. **ROS 2 multi-process**
   - Latency: from ~0.22 ms (320×240) to ~1.78 ms (1440×1080), strongly dependent on resolution.
   - CPU: for large images, pub/sub together use around 8%; for small images, under 1%.
   - Suitable when process isolation is required and millisecond-level latency and higher CPU cost are acceptable.

2. **ROS 2 intra-process**
   - Latency: ~0.02–0.03 ms at both resolutions.
   - CPU: ~0.31% for 320×240; ~1.38% for 1440×1080.
   - Eliminates IPC and extra scheduling overhead; this is the low-latency option within the ROS 2 framework.

3. **LibXR callback path (Publish → Callback Entry)**
   - Latency: sub-microsecond (0.4–0.5 µs), clearly lower than ROS intra-process.
   - Almost independent of resolution; dominated by LibXR’s internal path.

4. **LibXR full-frame copies inside the callback**
   - In this benchmark, each frame undergoes **three full copies** in the callback:
     - 1× business-buffer copy,
     - 1× `std::memcpy` timing,
     - 1× `FastCopy` timing.
   - This yields around 2% CPU usage under intentionally heavy copy pressure and does not represent an optimized real-world design.

5. **LibXR synchronous subscription path (Publish → Wait OK)**
   - ~1.06 ms at high resolution, ~0.10 ms at low resolution.
   - Affected by user-level synchronization and thread scheduling; this is a “cross-thread + synchronous” end-to-end figure.

This repository can serve as a base for further experiments, for example:

- Evaluating different threading models and synchronization strategies and their impact on subscription latency;
- Building longer processing pipelines and measuring end-to-end behavior;
- Validating copy strategies under different CPU/memory topologies (many-core, NUMA, etc.).