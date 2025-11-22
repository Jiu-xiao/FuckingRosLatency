#!/usr/bin/env bash
# 不用 -u，避免 setup.bash 里未定义变量导致中断
set -eo pipefail

# ===== 基本配置 =====
WS="${WS:-$HOME/ros2_ws}"
DURATION="${DURATION:-30}"          # 每种模式测试时长（秒）
RATE="${RATE:-30.0}"
WIDTH="${WIDTH:-1440}"
HEIGHT="${HEIGHT:-1080}"

LOG_DIR="$WS/logs"
BIN_DIR="$WS/install/image_latency_test/lib/image_latency_test"

mkdir -p "$LOG_DIR"

echo "Workspace: $WS"
echo "Log dir  : $LOG_DIR"
echo "Duration : $DURATION s"

# ===== 环境 / 工具检查 =====
if ! command -v pidstat >/dev/null 2>&1; then
  echo "[INFO] pidstat 未找到，自动安装 sysstat..."
  apt-get update -y
  apt-get install -y sysstat
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[ERROR] 找不到 /opt/ros/humble/setup.bash，确认镜像里有 ROS2 Humble。"
  exit 1
fi

cd "$WS"

ensure_binaries() {
  # 如果可执行文件不存在，自动 build 一次
  if [ ! -x "$BIN_DIR/image_publisher_node" ] || \
     [ ! -x "$BIN_DIR/image_subscriber_node" ] || \
     [ ! -x "$BIN_DIR/intra_process_image_latency" ]; then
    echo "[INFO] 找不到可执行文件，开始 colcon build image_latency_test ..."
    colcon build --packages-select image_latency_test
    echo "[INFO] build 完成。"
  fi
}

# ===== 小工具函数 =====

kill_all_test_procs() {
  echo "[INFO] 杀掉残留测试进程和 pidstat..."
  pkill -f "image_publisher_node" || true
  pkill -f "image_subscriber_node" || true
  pkill -f "intra_process_image_latency" || true
  pkill -f "pidstat -u" || true
}

analyze_latency() {
  local file="$1"
  local label="$2"

  if [ ! -f "$file" ]; then
    echo "[WARN] $label latency: 日志文件不存在: $file"
    return
  fi

  awk -v label="$label" '
    # 解析形如：
    # [multi_sub] frames=100 avg=3.307 ms last=2.315 ms
    # [intra]     frames=900 avg=0.932 ms last=1.680 ms
    /avg=/ {
      frames = avg = last = 0
      for (i = 1; i <= NF; i++) {
        if ($i ~ /^frames=/) {
          split($i, a, "="); frames = a[2] + 0
        } else if ($i ~ /^avg=/) {
          split($i, a, "="); avg = a[2] + 0
        } else if ($i ~ /^last=/) {
          split($i, a, "="); last = a[2] + 0
        }
      }
      if (avg > 0) {
        last_frames = frames
        last_avg = avg
      }
      if (last > 0) {
        if (n_last == 0 || last < min_last) min_last = last
        if (n_last == 0 || last > max_last) max_last = last
        n_last++
      }
    }

    END {
      if (last_avg > 0) {
        printf("[RESULT] %s latency: frames=%d avg=%.3f ms",
               label, last_frames, last_avg)
        if (n_last > 0) {
          printf(" min≈%.3f ms max≈%.3f ms", min_last, max_last)
        }
        printf("\n")
      } else {
        printf("[RESULT] %s latency: no data (file=%s)\n", label, FILENAME)
      }
    }
  ' "$file"
}

analyze_cpu() {
  local file="$1"
  local label="$2"

  if [ ! -f "$file" ]; then
    echo "[WARN] $label CPU: 日志文件不存在: $file"
    return
  fi

  # pidstat: 前 3 行是表头，从第 4 行开始是数据；第 8 列是 %CPU
  awk -v label="$label" '
    NR > 3 {
      sum += $8
      n++
    }
    END {
      if (n > 0) {
        printf("[RESULT] %s CPU: samples=%d avg=%.2f%%\n", label, n, sum/n)
      } else {
        printf("[RESULT] %s CPU: no data (file=%s)\n", label, FILENAME)
      }
    }
  ' "$file"
}

# ====== 1. 多进程 pub/sub 测试 ======

run_multi_process_test() {
  echo
  echo "====== 多进程 pub/sub 测试 开始 (duration=${DURATION}s) ======"

  kill_all_test_procs
  ensure_binaries

  local ts
  ts=$(date +%F_%H%M%S)

  local PUB_LOG="$LOG_DIR/pub_multi_${ts}.log"
  local SUB_LOG="$LOG_DIR/sub_multi_${ts}.log"
  local CPU_PUB_LOG="$LOG_DIR/cpu_pub_multi_${ts}.log"
  local CPU_SUB_LOG="$LOG_DIR/cpu_sub_multi_${ts}.log"

  echo "[INFO] pub log: $PUB_LOG"
  echo "[INFO] sub log: $SUB_LOG"

  "$BIN_DIR/image_publisher_node" \
    --ros-args -p publish_rate:="${RATE}" -p width:="${WIDTH}" -p height:="${HEIGHT}" \
    >"$PUB_LOG" 2>&1 &
  local PID_PUB=$!
  echo "[INFO] publisher PID=${PID_PUB}"

  "$BIN_DIR/image_subscriber_node" \
    >"$SUB_LOG" 2>&1 &
  local PID_SUB=$!
  echo "[INFO] subscriber PID=${PID_SUB}"

  sleep 2

  pidstat -u 1 -p "$PID_PUB" >"$CPU_PUB_LOG" &
  local PID_PIDSTAT_PUB=$!
  pidstat -u 1 -p "$PID_SUB" >"$CPU_SUB_LOG" &
  local PID_PIDSTAT_SUB=$!

  echo "[INFO] 多进程运行 ${DURATION} 秒..."
  sleep "$DURATION"

  echo "[INFO] 停止多进程 pub/sub 与 pidstat..."
  kill "$PID_PUB" "$PID_SUB" "$PID_PIDSTAT_PUB" "$PID_PIDSTAT_SUB" 2>/dev/null || true

  sleep 1

  echo
  echo "------ 多进程 测试结果 ------"
  analyze_latency "$SUB_LOG" "multi-process sub"
  analyze_cpu "$CPU_PUB_LOG" "multi-process pub"
  analyze_cpu "$CPU_SUB_LOG" "multi-process sub"
}

# ====== 2. 单进程 intra_process 测试 ======

run_intra_process_test() {
  echo
  echo "====== 进程内 intra_process 测试 开始 (duration=${DURATION}s) ======"

  kill_all_test_procs
  ensure_binaries

  local ts
  ts=$(date +%F_%H%M%S)

  local INTRA_LOG="$LOG_DIR/intra_${ts}.log"
  local CPU_INTRA_LOG="$LOG_DIR/cpu_intra_${ts}.log"

  echo "[INFO] intra log: $INTRA_LOG"

  "$BIN_DIR/intra_process_image_latency" \
    --ros-args -p publish_rate:="${RATE}" -p width:="${WIDTH}" -p height:="${HEIGHT}" \
    >"$INTRA_LOG" 2>&1 &
  local PID_INTRA=$!
  echo "[INFO] intra_process PID=${PID_INTRA}"

  sleep 2

  pidstat -u 1 -p "$PID_INTRA" >"$CPU_INTRA_LOG" &
  local PID_PIDSTAT_INTRA=$!

  echo "[INFO] 进程内运行 ${DURATION} 秒..."
  sleep "$DURATION"

  echo "[INFO] 停止 intra_process 与 pidstat..."
  kill "$PID_INTRA" "$PID_PIDSTAT_INTRA" 2>/dev/null || true

  sleep 1

  echo
  echo "------ 进程内 测试结果 ------"
  analyze_latency "$INTRA_LOG" "intra-process"
  analyze_cpu "$CPU_INTRA_LOG" "intra-process"
}

# ===== 主流程 =====

kill_all_test_procs

run_multi_process_test
run_intra_process_test

echo
echo "====== 全部测试完成，详细日志在 $LOG_DIR 下 ======"
