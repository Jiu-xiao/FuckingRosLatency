#!/usr/bin/env bash
# 不用 -u，避免 setup.bash 里未定义变量导致中断
set -eo pipefail

# ===== 基本配置 =====
WS="${WS:-$HOME/ros2_ws}"
LOG_DIR="$WS/logs"
BIN_PATH="$WS/build/libxr_tp_test/libxr_tp_test"

mkdir -p "$LOG_DIR"

echo "Workspace: $WS"
echo "Log dir  : $LOG_DIR"

# ===== 环境 / 工具检查 =====
if ! command -v pidstat >/dev/null 2>&1; then
  echo "[INFO] pidstat 未找到，自动安装 sysstat..."
  apt-get update -y
  apt-get install -y sysstat
fi

# libxr_tp_test 是 colcon 包的话，build 需要 ROS 环境
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
  if [ ! -x "$BIN_PATH" ]; then
    echo "[INFO] 找不到可执行文件 $BIN_PATH，开始 colcon build libxr_tp_test ..."
    colcon build --packages-select libxr_tp_test
    echo "[INFO] build 完成。"
  fi
}

kill_all_test_procs() {
  echo "[INFO] 杀掉残留 libxr 测试进程和 pidstat..."
  pkill -f "libxr_tp_test" || true
  pkill -f "pidstat -u" || true
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

run_libxr_test() {
  echo
  echo "====== LibXR image latency 测试开始（程序内部会依次测 1440x1080 / 320x240） ======"

  kill_all_test_procs
  ensure_binaries

  local ts
  ts=$(date +%F_%H%M%S)

  local LIBXR_LOG="$LOG_DIR/libxr_${ts}.log"
  local CPU_LIBXR_LOG="$LOG_DIR/cpu_libxr_${ts}.log"

  echo "[INFO] libxr log: $LIBXR_LOG"

  # 启动 libxr 基准测试
  "$BIN_PATH" >"$LIBXR_LOG" 2>&1 &
  local PID_LIBXR=$!
  echo "[INFO] libxr_tp_test PID=${PID_LIBXR}"

  sleep 1

  # pidstat 统计整个 libxr 进程生命周期的 CPU
  pidstat -u 1 -p "$PID_LIBXR" >"$CPU_LIBXR_LOG" &
  local PID_PIDSTAT_LIBXR=$!

  # 等待 libxr 测试结束
  wait "$PID_LIBXR" || true

  # 停掉 pidstat
  kill "$PID_PIDSTAT_LIBXR" 2>/dev/null || true

  echo
  echo "------ LibXR image latency 测试结果（从日志中抽取）------"
  # 把 [RESULT] 行抽出来再打印一遍，方便 CI 控制台查看
  if grep -q "\[RESULT\]" "$LIBXR_LOG"; then
    grep "\[RESULT\]" "$LIBXR_LOG"
  else
    echo "[WARN] 没在 $LIBXR_LOG 里找到 [RESULT] 行，请检查程序输出。"
  fi

  echo
  echo "------ LibXR CPU 统计 ------"
  analyze_cpu "$CPU_LIBXR_LOG" "libxr_tp_test"
}

# ===== 主流程 =====

run_libxr_test

echo
echo "====== LibXR 测试完成，详细日志在 $LOG_DIR 下 ======"
