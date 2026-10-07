#ifndef MYMIT_ROBOT_HARDWARE_PERIODIC_MIT_SENDER_HPP_
#define MYMIT_ROBOT_HARDWARE_PERIODIC_MIT_SENDER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>

#include "hardware/dm1_mit_interface.hpp"

namespace dm1_hardware
{
// 控制计算/通信短暂失败时，独立以125Hz持续重发最近一组已校验命令。
// 不因超时或发送失败自动失能；电机故障保护和显式停机由硬件桥负责。
class PeriodicMitSender
{
public:
  using Clock = std::chrono::steady_clock;
  using Frames = std::array<MitFrame, kNumJoints>;

  PeriodicMitSender(MitTransport & transport, const std::atomic_bool & stop_requested)
  : transport_(transport), stop_requested_(stop_requested), worker_([this] {run();}) {}

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
    return true;
  }

  // 返回后没有在途MIT发送；调用方此后才能失能或重新使能。
  void pause()
  {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    std::lock_guard<std::mutex> lock(state_mutex_);
    frames_.reset();
    active_ = false;
  }

  void shutdown()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stopping_ = true;
    }
    wake_.notify_one();
    if (worker_.joinable()) {worker_.join();}
  }

  std::uint64_t failedFrames() const noexcept {return failed_frames_.load();}

private:
  static constexpr auto kPeriod = std::chrono::milliseconds(8);

  void run()
  {
    auto next = Clock::now();
    for (;;) {
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        wake_.wait_until(lock, next, [this] {return stopping_;});
        if (stopping_) {return;}
      }
      // 与pause串行，避免用户锁定/电机保护失能之后又发出旧动作。
      std::lock_guard<std::mutex> io_lock(io_mutex_);
      Frames frames;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (stopping_) {return;}
        if (!frames_) {
          next = Clock::now() + kPeriod;
          continue;
        }
        frames = *frames_;
      }
      for (const auto & frame : frames) {
        // 外部显式退出请求，不依赖控制线程是否正在计算。
        if (stop_requested_.load()) {
          transport_.disableAll();
          return;
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
      next += kPeriod;
      if (next < Clock::now()) {next = Clock::now() + kPeriod;}
    }
  }

  MitTransport & transport_;
  const std::atomic_bool & stop_requested_;
  std::mutex state_mutex_;
  std::mutex io_mutex_;
  std::condition_variable wake_;
  std::optional<Frames> frames_;
  bool stopping_ = false;
  bool active_ = false;
  std::atomic<std::uint64_t> failed_frames_{0};
  std::thread worker_;
};
}  // namespace dm1_hardware

#endif
