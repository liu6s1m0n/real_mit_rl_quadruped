#ifndef MYMIT_ROBOT_HARDWARE_PERIODIC_MIT_SENDER_HPP_
#define MYMIT_ROBOT_HARDWARE_PERIODIC_MIT_SENDER_HPP_

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
// 控制计算/通信短暂失败时，两条总线独立以500Hz续发最近一组已校验命令。
// 不因超时或发送失败自动失能；电机故障保护和显式停机由硬件桥负责。
class PeriodicMitSender
{
public:
  using Clock = std::chrono::steady_clock;
  using Frames = std::array<MitFrame, kNumJoints>;

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

  void arm()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_ = true;
  }

  bool publish(const Frames & frames)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!active_ || stopping_ || stop_requested_.load()) {return false;}
    frames_ = frames;
    published_at_ = Clock::now();
    return true;
  }

  // 返回后没有在途MIT发送；调用方此后才能失能或重新使能。
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

  std::uint64_t failedFrames() const noexcept {return failed_frames_.load();}

private:
  static constexpr auto kPeriod = std::chrono::milliseconds(2);

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
        imu_log::print(imu_log::Level::Info,
          "[DM1][TX-SCHEDULE] bus=%u batch_hz=%.1f overruns=%llu command_age_ms=%.3f monitor_only=true\n",
          static_cast<unsigned int>(bus), batches / window_s,
          static_cast<unsigned long long>(overruns),
          std::chrono::duration<double, std::milli>(finished - published_at).count());
        report_at = finished;
        batches = overruns = 0;
      }
    }
  }

  MitTransport & transport_;
  const std::atomic_bool & stop_requested_;
  std::mutex state_mutex_;
  std::array<std::mutex, 2> io_mutex_;
  std::condition_variable wake_;
  std::optional<Frames> frames_;
  Clock::time_point published_at_{};
  bool stopping_ = false;
  bool active_ = false;
  std::atomic<std::uint64_t> failed_frames_{0};
  std::atomic_bool stop_disabled_{false};
  std::array<std::thread, 2> workers_;
};
}  // namespace dm1_hardware

#endif
