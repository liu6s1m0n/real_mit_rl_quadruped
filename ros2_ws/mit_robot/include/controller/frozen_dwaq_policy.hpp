/**
 * @file frozen_dwaq_policy.hpp
 * 换模型要修改的地方：模型编号。
 * @brief Dependency-free deployment adapter for the frozen DM1 model_4210.
 */
#ifndef MYMIT_ROBOT_CONTROLLER_FROZEN_DWAQ_POLICY_HPP_
#define MYMIT_ROBOT_CONTROLLER_FROZEN_DWAQ_POLICY_HPP_

#include "controller/RlPolicy.hpp"

/**
 * @brief Runs the model_4210 DreamWaQ posterior-mean actor in C++.
 *
 * CMake generates the weight header from the original PyTorch checkpoint. The
 * inference equations intentionally mirror ActorCritic_DWAQ.act_inference:
 * VAE history encoder -> posterior means -> actor MLP. No MPC/WBC code is
 * shared or changed by this adapter.
 */
class FrozenDwaqPolicy final : public RlPolicy
{
public:
  bool infer(
    const std::array<float, kRlObservationSize> & observation,
    const std::array<float, kRlObservationSize * kRlHistoryLength> & history,
    std::array<float, kRlActionSize> & action) override;

  RlPolicyMetadata metadata() const override;
};

#endif  // MYMIT_ROBOT_CONTROLLER_FROZEN_DWAQ_POLICY_HPP_
