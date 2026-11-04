#!/usr/bin/env bash
# 配置 DM1 的 SocketCAN FD 接口。
#
# 用法：sudo setup_dm1_can.sh [接口名...]，默认配置 can0 和 can1。
# 每个接口设置为仲裁域 1 Mbps、数据域 5 Mbps 的 CAN FD，开启 bus-off 自动恢复，
# 并把发送队列限制为 32 帧。
set -euo pipefail

readonly ARBITRATION_BITRATE=1000000  # 仲裁域波特率，bit/s。
readonly DATA_BITRATE=5000000         # 数据域波特率，bit/s。
readonly SAMPLE_POINT=0.75            # 仲裁域采样点。
readonly DATA_SAMPLE_POINT=0.875      # 数据域采样点。

if [[ ${EUID} -ne 0 ]]; then
  echo "请使用 sudo 运行：sudo $0" >&2
  exit 1
fi

if (($# == 0)); then
  interfaces=(can0 can1)
else
  interfaces=("$@")
fi

for interface in "${interfaces[@]}"; do
  if [[ ! ${interface} =~ ^can[0-9]+$ ]]; then
    echo "无效的 SocketCAN 接口名：${interface}" >&2
    exit 2
  fi
  if ! ip link show dev "${interface}" >/dev/null 2>&1; then
    echo "找不到 SocketCAN 接口：${interface}" >&2
    exit 3
  fi

  ip link set dev "${interface}" down
  ip link set dev "${interface}" type can \
    bitrate "${ARBITRATION_BITRATE}" sample-point "${SAMPLE_POINT}" \
    dbitrate "${DATA_BITRATE}" dsample-point "${DATA_SAMPLE_POINT}" \
    fd on loopback off
  # 某个电机掉线/CAN 错误会让控制器持续重传，bus-off 后发送队列不再清空，
  # 表现为 write() 返回 ENOBUFS(105)。restart-ms 让控制器自动恢复。
  # 部分驱动不支持该属性，失败不致命。
  ip link set dev "${interface}" type can restart-ms 100 2>/dev/null || \
    echo "${interface}: 该驱动不支持 restart-ms，跳过自动恢复设置" >&2
  # 控制帧具有时效性：小队列只吸收短暂调度抖动，故障时不积压数百帧旧命令。
  ip link set dev "${interface}" txqueuelen 32
  ip link set dev "${interface}" up

  echo "${interface}: FDCAN 已启动（仲裁域 1 Mbps/75%，数据域 5 Mbps/87.5%）"
  # 统计里的 bus-error / error-passive / bus-off 计数是排查 ENOBUFS 的关键。
  ip -details -statistics link show dev "${interface}"
done
