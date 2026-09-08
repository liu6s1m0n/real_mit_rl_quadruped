#!/usr/bin/env bash
set -euo pipefail

readonly ARBITRATION_BITRATE=1000000
readonly DATA_BITRATE=1000000
readonly SAMPLE_POINT=0.75

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
    dbitrate "${DATA_BITRATE}" dsample-point "${SAMPLE_POINT}" \
    fd on loopback off
  ip link set dev "${interface}" txqueuelen 100
  ip link set dev "${interface}" up

  echo "${interface}: FDCAN 已启动（仲裁域 1 Mbps，数据域 1 Mbps，采样点 75%）"
  ip -details link show dev "${interface}"
done
