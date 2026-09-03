/**
 * @file RlPolicy.hpp
 * @brief DM1 RL 部署边界；推理实现与控制核心解耦。
 *
 * 这里不依赖 PyTorch/Isaac/CAN。部署程序可通过 CallbackRlPolicy 注入
 * TorchScript、ONNX 或远程推理实现；控制器只接受固定的 45/270 维接口。
 */
#ifndef MYMIT_ROBOT_CONTROLLER_RL_POLICY_HPP_
#define MYMIT_ROBOT_CONTROLLER_RL_POLICY_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

constexpr std::size_t kRlObservationSize = 45;
constexpr std::size_t kRlHistoryLength = 6;
constexpr std::size_t kRlActionSize = 12;
inline constexpr char kDm1FlatCheckpointSha256[] =
  "aaafcccd6aa6a65d247051f7f7f419b518314e5a7ec934454267d41d5a753d1f";
inline constexpr char kDm1StairsCheckpointSha256[] =
  "cc653d0b1b60459e45a0ce7b3face8a927f1fb4d8113f4bef7366c44ca81ec6d";

enum class ControlMode : std::uint8_t;

struct RlPolicyMetadata
{
  std::string name;
  std::string checkpoint_sha256;
  bool supports_stairs = false;
  bool frozen = false;
  bool uses_vae_posterior_mean = false;
};

class RlPolicy
{
public:
  virtual ~RlPolicy() = default;
  virtual bool infer(
    const std::array<float, kRlObservationSize> & observation,
    const std::array<float, kRlObservationSize * kRlHistoryLength> & history,
    std::array<float, kRlActionSize> & action) = 0;
  virtual RlPolicyMetadata metadata() const = 0;
};

class CallbackRlPolicy final : public RlPolicy
{
public:
  using Callback = std::function<bool(
      const std::array<float, kRlObservationSize> &,
      const std::array<float, kRlObservationSize * kRlHistoryLength> &,
      std::array<float, kRlActionSize> &)>;

  CallbackRlPolicy(Callback callback, RlPolicyMetadata metadata)
  : callback_(std::move(callback)), metadata_(std::move(metadata)) {}

  bool infer(
    const std::array<float, kRlObservationSize> & observation,
    const std::array<float, kRlObservationSize * kRlHistoryLength> & history,
    std::array<float, kRlActionSize> & action) override
  {
    return callback_ && callback_(observation, history, action);
  }

  RlPolicyMetadata metadata() const override {return metadata_;}

private:
  Callback callback_;
  RlPolicyMetadata metadata_;
};

using RlPolicyPtr = std::shared_ptr<RlPolicy>;

#endif  // MYMIT_ROBOT_CONTROLLER_RL_POLICY_HPP_
