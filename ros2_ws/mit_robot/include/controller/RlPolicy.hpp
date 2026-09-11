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
/*换模型要修改的地方：必须与 Isaac 训练配置 init_state.default_joint_angles 一致*/
inline constexpr std::array<float, kRlActionSize> kDm1RlDefaultJointPosition{
  0.0F, -0.520F, 1.330F,
  0.0F, -0.520F, 1.330F,
  0.0F, -0.520F, 1.330F,
  0.0F, -0.520F, 1.330F};
inline constexpr float kDm1RlDefaultBodyHeight = 0.38F;
// Default deployment checkpoint: model_4210. model_4245 is selectable as an
// alternate yaw-recovery checkpoint through the MuJoCo CLI.
// /home/simon/RL_Robot/frozen_models/dm1_trot_directional_hip_refine_model4210/model_4210.pt
inline constexpr char kDm1FlatCheckpointSha256[] =
  "3f4c65ec73d435b7fb16ad24bb240114c01b79502c6eb1037daa887740a0e06b";
inline constexpr char kDm1YawRecoveryCheckpointSha256[] =
  "65700456169a3d7d17ab279fb0c48b6b306717345f8ae90dbc2218e1384a6f34";
// Previous checkpoint (commented, not active): model_3610
// SHA256 cf6f88a2c7be9365dbe3e62c9a4c022d37d88f7fd692373842c32280f7e88421
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
