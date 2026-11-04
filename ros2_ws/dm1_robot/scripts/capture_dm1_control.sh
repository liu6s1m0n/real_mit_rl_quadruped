#!/usr/bin/env bash
# 采集一次按键 3/4（对角单步）的真机离线诊断数据。
#
# 用法：capture_dm1_control.sh 3|4 [hardware_main 额外参数...]
# 以键盘控制模式启动 hardware_main，打开全量诊断，把 CSV、控制器日志、启动命令
# 和标定文件副本一起保存到 dm1_diagnostics/offline/key<按键>_<时间>_<PID>/。
# 必须在交互式终端中运行，按键由操作员手动输入。
set -u

if [[ $# -lt 1 || "$1" != "3" && "$1" != "4" ]]; then
  echo "用法: $0 3 [hardware_main参数...]" >&2
  echo "     $0 4 [hardware_main参数...]" >&2
  exit 2
fi

key="$1"
shift
# 路径均相对脚本位置推导：脚本在 dm1_robot/scripts，工作空间为其上两级。
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws_dir="$(cd "$script_dir/../.." && pwd)"
build_dir="$ws_dir/build/dm1_robot"
binary="$build_dir/hardware_main"
calibration="$ws_dir/dm1_robot/config/hardware/dm1_hardware_calibration.txt"
if [[ ! -x "$binary" ]]; then
  echo "找不到正式程序: $binary" >&2
  echo "先进入 $ws_dir，再执行: CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --packages-select dm1_robot --cmake-target hardware_main --parallel-workers 1 --executor sequential" >&2
  exit 1
fi
if [[ ! -f "$calibration" ]]; then
  echo "找不到标定文件: $calibration" >&2
  exit 1
fi
if [[ ! -t 0 ]]; then
  echo "必须在交互式终端中运行，脚本不会自动发送按键 3/4。" >&2
  exit 1
fi

# 每次运行使用独立目录，避免覆盖之前的数据。
stamp="$(date +%Y%m%d_%H%M%S)"
run_dir="$ws_dir/dm1_diagnostics/offline/key${key}_${stamp}_$$"
mkdir -p "$run_dir"

cat > "$run_dir/README.txt" <<INFO
按键 ${key} 离线控制日志
开始时间: $(date --iso-8601=seconds)

操作:
  U  解锁
  1  站立并等待稳定
  ${key}  执行一次对角原地踏步
  等待动作完成后可再次按 ${key}
  0  失能
  Esc  退出程序

文件:
  control.csv       IMU、状态估计、电机反馈/命令
  controller.log    MPC/WBC 全量诊断文本
  command.txt      本次启动命令
INFO

printf '%q ' "$binary" --calibration "$calibration" --keyboard-control "$@" > "$run_dir/command.txt"
printf '\n' >> "$run_dir/command.txt"
cp -f "$calibration" "$run_dir/dm1_hardware_calibration.txt"

# DM1_LOG_DIR 指定 CSV 输出目录，其余变量打开控制器诊断输出。
export DM1_LOG_DIR="$run_dir"
export DM1_MPC_DIAGNOSTICS=1
export DM1_CONTROL_DIAGNOSTICS=1
export DM1_CONTROL_DIAGNOSTICS_FULL=1

cat <<INFO
dm1 按键 ${key} 采集目录: $run_dir
日志会写入: $run_dir/controller.log
请按 U -> 1，稳定后按 ${key} 一次；动作结束后按 0，再按 Esc 退出。
INFO

# 允许程序以非 0 退出码结束，仍要记录退出码。
set +e
"$binary" --calibration "$calibration" --keyboard-control "$@" > "$run_dir/controller.log" 2>&1
status=$?
set -e

echo "程序退出码: $status" | tee "$run_dir/exit_status.txt"
echo "日志目录: $run_dir"
exit "$status"
