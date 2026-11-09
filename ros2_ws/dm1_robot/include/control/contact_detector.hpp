/*! @file contact_detector.hpp
 *  @brief 计划支撑腿的真实触地检测（晚落地处理）。
 *
 *  2026-11-09 实机后腿比计划晚落地 20–35 ms：MPC/WBC 在计划落地时刻就把该腿当作支撑，
 *  脚还在空中时机尾下沉，真正着地后 30 ms 内 WBC 竖直力冲到约 100 N。
 *  本模块只做判断：计划进入支撑后，足端竖直力超过阈值，或足端已接近地面且不再下降，
 *  连续若干周期即认为触地；超过最长等待时间仍未满足也按触地处理，避免一直悬空等待。
 *  计划摆动的腿一律为未触地（不做提前触地判断）。
 *  离线数据里前腿基本准时落地、落地初期只承重约 15–35 N，足端高度/速度信号在毫米级分不清“将落未落”和“刚落地”，
 *  所以缺省只对后腿用足端力判断（heightThreshold 设为 0 即关闭高度/速度条件）。
 */

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

/// 触地检测参数，task 文件 contactDetection 段。
struct ContactDetectionSettings
{
  int mode = 0;                     ///< 0 关闭（触地=计划），1 影子（只记录），2 参与控制。
  double force_threshold = 45.0;    ///< 足端竖直力阈值，N。
  double height_threshold = 0.004;  ///< 足端高出地面估计不超过此值视为接近地面，m。
  double velocity_threshold = 0.05;  ///< 足端世界竖直速度高于 -此值视为已停止下降，m/s。
  int confirm_ticks = 2;            ///< 条件需连续满足的控制周期数。
  double max_delay = 0.06;          ///< 计划落地后最长等待时间，s。
  double probe_velocity = 0.1;      ///< 等待期间足端继续下探的速度，m/s（仅 mode 2）。
  /// 参与检测的腿（调用方接触点序）；不参与的腿触地即计划。
  std::array<bool, 4> enabled{{true, true, true, true}};
};

/// 一条腿一个控制周期的检测输入。
struct ContactSignal
{
  bool planned_stance = false;  ///< 计划是否支撑。
  double force_z = 0.0;         ///< 由关节力矩反推的足端竖直力（地面对足端，向上为正），N。
  double height = 0.0;          ///< 足端高度减去该腿地面高度估计，m。
  double velocity_z = 0.0;      ///< 足端世界竖直速度，m/s。
};

/**
 * @brief 四条腿的触地状态机，按调用方的接触点序存放。
 */
class ContactDetector
{
public:
  static constexpr std::size_t kLegs = 4;

  explicit ContactDetector(ContactDetectionSettings settings = {})
  : settings_(settings) {}

  const ContactDetectionSettings & settings() const noexcept {return settings_;}

  /// 清空状态：之后计划支撑的腿重新按晚落地等待。
  void reset() noexcept
  {
    detected_.fill(false);
    wait_time_.fill(0.0);
    confirm_.fill(0);
  }

  /// 把计划支撑的腿直接标记为已触地（非踏步模式下使用，四脚本来就在地上）。
  void acceptPlanned(const std::array<bool, kLegs> & planned_stance) noexcept
  {
    for (std::size_t leg = 0; leg < kLegs; ++leg) {
      detected_[leg] = planned_stance[leg];
      wait_time_[leg] = 0.0;
      confirm_[leg] = 0;
    }
  }

  /// 推进一个控制周期，返回各腿是否已真实触地。
  const std::array<bool, kLegs> & update(
    const std::array<ContactSignal, kLegs> & signals, double period) noexcept
  {
    for (std::size_t leg = 0; leg < kLegs; ++leg) {
      const auto & signal = signals[leg];
      if (!settings_.enabled[leg]) {
        detected_[leg] = signal.planned_stance;
        continue;
      }
      if (!signal.planned_stance) {
        detected_[leg] = false;
        wait_time_[leg] = 0.0;
        confirm_[leg] = 0;
        continue;
      }
      if (detected_[leg]) {continue;}
      wait_time_[leg] += std::max(period, 0.0);
      const bool loaded = signal.force_z > settings_.force_threshold;
      const bool landed = settings_.height_threshold > 0.0 &&
        signal.height < settings_.height_threshold &&
        signal.velocity_z > -settings_.velocity_threshold;
      confirm_[leg] = (loaded || landed) ? confirm_[leg] + 1 : 0;
      if (confirm_[leg] >= settings_.confirm_ticks || wait_time_[leg] >= settings_.max_delay) {
        detected_[leg] = true;
      }
    }
    return detected_;
  }

  const std::array<bool, kLegs> & detected() const noexcept {return detected_;}

  /// 计划支撑但尚未触地（等待/下探中）。
  bool waiting(std::size_t leg, bool planned_stance) const noexcept
  {
    return planned_stance && !detected_[leg];
  }

private:
  ContactDetectionSettings settings_;
  std::array<bool, kLegs> detected_{};
  std::array<double, kLegs> wait_time_{};
  std::array<int, kLegs> confirm_{};
};
