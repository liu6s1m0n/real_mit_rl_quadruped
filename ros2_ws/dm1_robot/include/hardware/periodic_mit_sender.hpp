/*! @file periodic_mit_sender.hpp
 *  @brief 按总线独立的 500 Hz MIT 命令续发线程。
 */
#ifndef DM1_ROBOT_HARDWARE_PERIODIC_MIT_SENDER_HPP_
#define DM1_ROBOT_HARDWARE_PERIODIC_MIT_SENDER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <system_error>
#include <thread>

#include "hardware/dm1_mit_interface.hpp"

namespace dm1_hardware
{
/**
 * @brief 两条 CAN 总线各一个线程，以 500 Hz 发送最近一批已校验的 MIT 命令。
 *
 * 控制计算/通信短暂失败时继续续发最近一组命令。不因超时或发送失败自动失能；
 * 电机故障保护和显式停机由硬件桥负责。只有收到外部停止请求时才统一失能一次。
 */
class PeriodicMitSender
{
public:
  using Clock = std::chrono::steady_clock;
  using Frames = std::array<MitFrame, kNumJoints>;  ///< 一批 12 帧命令。

  /// 立即启动两条总线的发送线程；线程创建失败时回收已启动的线程并重新抛出。
  PeriodicMitSender(MitTransport & transport, const std::atomic_bool & stop_requested)
  : transport_(transport), stop_requested_(stop_requested)
  {
    try {
      for (std::uint8_t bus = 0; bus < workers_.size(); ++bus) {
        workers_[bus] = std::thread([this, bus] {run(bus);});
      }
    } catch (const std::system_error &) {
      shutdown();
      throw;
    }
  }

  ~PeriodicMitSender() {shutdown();}

  /// 允许 publish() 写入新命令；电机使能完成后调用。
  void arm()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_ = true;
  }

  /// 替换待发送的命令批次；未 arm、正在停止或已请求退出时返回 false。
  bool publish(const Frames & frames)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!active_ || stopping_ || stop_requested_.load()) {return false;}
    frames_ = frames;
    published_at_ = Clock::now();
    return true;
  }

  /// 清空命令并停止发送；返回后没有在途 MIT 发送，调用方此后才能失能或重新使能。
  void pause()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      frames_.reset();
      active_ = false;
    }
    // 先停止新批次，再等待在途帧，避免500Hz续发导致双路锁饥饿。
    std::scoped_lock io_lock(io_mutex_[0], io_mutex_[1]);
  }

  /// 停止并 join 两个发送线程，可重复调用。
  void shutdown()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stopping_ = true;
    }
    wake_.notify_all();
    for (auto & worker : workers_) {
      if (worker.joinable()) {worker.join();}
    }
  }

  /// 累计发送失败的帧数。
  std::uint64_t failedFrames() const noexcept {return failed_frames_.load();}

private:
  static constexpr auto kPeriod = std::chrono::milliseconds(2);  ///< 发送周期，500 Hz。

  /// 单条总线的发送循环：每周期取最新批次，只发送属于本总线的帧，每秒打印一次统计。
  void run(std::uint8_t bus)
  {
    auto next = Clock::now();
    auto report_at = next;
    std::uint64_t batches = 0, overruns = 0;
    for (;; ) {
      if (stop_requested_.load()) {
        // 先等待两路在途发送结束，再且仅执行一次显式停机。
        pause();
        if (!stop_disabled_.exchange(true)) {transport_.disableAll();}
        return;
      }
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        wake_.wait_until(lock, next, [this] {return stopping_;});
        if (stopping_) {return;}
      }
      // 与pause串行，避免用户锁定/电机保护失能之后又发出旧动作。
      std::unique_lock<std::mutex> io_lock(io_mutex_[bus]);
      Frames frames;
      Clock::time_point published_at;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (stopping_) {return;}
        if (!frames_) {
          next = Clock::now() + kPeriod;
          continue;
        }
        frames = *frames_;
        published_at = published_at_;
      }
      for (const auto & frame : frames) {
        if (frame.bus != bus) {continue;}
        // 外部显式退出请求，不依赖控制线程是否正在计算。
        if (stop_requested_.load()) {
          break;
        }
        bool sent = false;
        try {
          sent = transport_.sendMit(frame);
        } catch (...) {
          // 传输异常按发送失败处理，仍尝试其余电机并在下一轮重试。
        }
        if (!sent) {++failed_frames_;}
        // 留出USB-CAN上线时间；不立即重发整批以免拥塞进一步加重。
        std::this_thread::sleep_for(std::chrono::microseconds(250));
      }
      io_lock.unlock();
      ++batches;
      next += kPeriod;
      const auto finished = Clock::now();
      // 不追赶旧批次，也不额外等待2ms；下一轮仍只发送最新命令。
      if (next < finished) {next = finished; ++overruns;}
      const double window_s = std::chrono::duration<double>(finished - report_at).count();
      if (window_s >= 1.0) {
        imu_log::print(
          imu_log::Level::Info,
          "[DM1][TX-SCHEDULE] bus=%u batch_hz=%.1f overruns=%llu command_age_ms=%.3f monitor_only=true\n",
          static_cast<unsigned int>(bus), batches / window_s,
          static_cast<unsigned long long>(overruns),
          std::chrono::duration<double, std::milli>(finished - published_at).count());
        report_at = finished;
        batches = overruns = 0;
      }
    }
  }

  MitTransport & transport_;                 ///< 底层发送接口。
  const std::atomic_bool & stop_requested_;  ///< 外部退出请求。
  std::mutex state_mutex_;                   ///< 保护 frames_、published_at_、stopping_、active_。
  std::array<std::mutex, 2> io_mutex_;       ///< 每条总线一把，标记该总线有在途发送。
  std::condition_variable wake_;             ///< 用于 shutdown() 唤醒等待中的线程。
  std::optional<Frames> frames_;             ///< 最新待发送批次；为空时不发送。
  Clock::time_point published_at_{};         ///< 最新批次的发布时间。
  bool stopping_ = false;                    ///< 正在关闭发送线程。
  bool active_ = false;                      ///< 是否允许发布命令。
  std::atomic<std::uint64_t> failed_frames_{0};  ///< 累计发送失败帧数。
  std::atomic_bool stop_disabled_{false};        ///< 确保退出时只调用一次 disableAll()。
  std::array<std::thread, 2> workers_;           ///< 两条总线的发送线程。
};
}  // namespace dm1_hardware

#endif
