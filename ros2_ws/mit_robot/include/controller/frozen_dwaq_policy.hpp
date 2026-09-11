/**
 * @file frozen_dwaq_policy.hpp
 * @brief Dependency-free deployment adapter for the frozen DM1 RL models.
 */
#ifndef MYMIT_ROBOT_CONTROLLER_FROZEN_DWAQ_POLICY_HPP_
#define MYMIT_ROBOT_CONTROLLER_FROZEN_DWAQ_POLICY_HPP_

#include <cstdint>

#include "controller/RlPolicy.hpp"

/**
 * @brief Runs a selected DreamWaQ posterior-mean actor in C++.
 *
 * CMake generates the weight header from the original PyTorch checkpoint. The
 * inference equations intentionally mirror ActorCritic_DWAQ.act_inference:
 * VAE history encoder -> posterior means -> actor MLP. No MPC/WBC code is
 * shared or changed by this adapter.
 */
enum class FrozenDwaqModel : std::uint8_t
{
  Model4210,
  Model4245
};

class FrozenDwaqPolicy final : public RlPolicy
{
public:
  explicit FrozenDwaqPolicy(
    FrozenDwaqModel model = FrozenDwaqModel::Model4210) noexcept
  : model_(model) {}

  bool infer(
    const std::array<float, kRlObservationSize> & observation,
    const std::array<float, kRlObservationSize * kRlHistoryLength> & history,
    std::array<float, kRlActionSize> & action) override;

  RlPolicyMetadata metadata() const override;

private:
  FrozenDwaqModel model_;
};

#endif  // MYMIT_ROBOT_CONTROLLER_FROZEN_DWAQ_POLICY_HPP_
