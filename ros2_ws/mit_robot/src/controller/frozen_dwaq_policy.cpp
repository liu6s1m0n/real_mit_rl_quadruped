#include "controller/frozen_dwaq_policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

/*换模型要修改的地方：生成头文件名*/
#include "dm1_policy_3610.hpp"

namespace
{
struct DenseLayer
{
  const float * weights;
  const float * bias;
  std::size_t output_size;
  std::size_t input_size;
};

void dense(const DenseLayer & layer, const float * input, float * output)
{
  for (std::size_t row = 0; row < layer.output_size; ++row) {
    float value = layer.bias[row];
    for (std::size_t column = 0; column < layer.input_size; ++column) {
      value += layer.weights[row * layer.input_size + column] * input[column];
    }
    output[row] = value;
  }
}

float elu(float value) noexcept
{
  return value >= 0.0F ? value : std::expm1(value);
}

template<std::size_t Size>
void applyElu(std::array<float, Size> & values) noexcept
{
  for (float & value : values) {value = elu(value);}
}

/*换模型要修改的地方：生成头文件 namespace*/
const DenseLayer kEncoder0{
  dm1_policy_3610::k_vae_encoder_encoder_0_weight.data(),
  dm1_policy_3610::k_vae_encoder_encoder_0_bias.data(), 128, 270};
const DenseLayer kEncoder2{
  dm1_policy_3610::k_vae_encoder_encoder_2_weight.data(),
  dm1_policy_3610::k_vae_encoder_encoder_2_bias.data(), 64, 128};
const DenseLayer kLatentMu{
  dm1_policy_3610::k_vae_latent_mu_weight.data(),
  dm1_policy_3610::k_vae_latent_mu_bias.data(), 16, 64};
const DenseLayer kVelocityMu{
  dm1_policy_3610::k_vae_vel_mu_weight.data(),
  dm1_policy_3610::k_vae_vel_mu_bias.data(), 3, 64};
const DenseLayer kActor0{
  dm1_policy_3610::k_actor_0_weight.data(),
  dm1_policy_3610::k_actor_0_bias.data(), 512, 64};
const DenseLayer kActor2{
  dm1_policy_3610::k_actor_2_weight.data(),
  dm1_policy_3610::k_actor_2_bias.data(), 256, 512};
const DenseLayer kActor4{
  dm1_policy_3610::k_actor_4_weight.data(),
  dm1_policy_3610::k_actor_4_bias.data(), 128, 256};
const DenseLayer kActor6{
  dm1_policy_3610::k_actor_6_weight.data(),
  dm1_policy_3610::k_actor_6_bias.data(), 12, 128};
}  // namespace

bool FrozenDwaqPolicy::infer(
  const std::array<float, kRlObservationSize> & observation,
  const std::array<float, kRlObservationSize * kRlHistoryLength> & history,
  std::array<float, kRlActionSize> & action)
{
  for (const float value : observation) {
    if (!std::isfinite(value) || std::abs(value) > 100.0F) {return false;}
  }
  for (const float value : history) {
    if (!std::isfinite(value) || std::abs(value) > 100.0F) {return false;}
  }
  // DM1 training initializes obs_hist_buf by repeating the first observation
  // across all six frames. The existing FSM deliberately starts its private
  // history at zero, so repair only this deployment-boundary case here rather
  // than changing the locomotion/controller code.
  std::array<float, kRlObservationSize * kRlHistoryLength> deployment_history = history;
  bool history_prefix_is_zero = true;
  for (std::size_t index = 0; index < deployment_history.size() - kRlObservationSize;
    ++index)
  {
    if (deployment_history[index] != 0.0F) {
      history_prefix_is_zero = false;
      break;
    }
  }
  if (history_prefix_is_zero) {
    for (std::size_t frame = 0; frame < kRlHistoryLength; ++frame) {
      std::copy(
        observation.begin(), observation.end(),
        deployment_history.begin() + frame * kRlObservationSize);
    }
  }
  std::array<float, 128> encoder_hidden{};
  std::array<float, 64> encoded{};
  dense(kEncoder0, deployment_history.data(), encoder_hidden.data());
  applyElu(encoder_hidden);
  dense(kEncoder2, encoder_hidden.data(), encoded.data());

  std::array<float, 16> latent_mu{};
  std::array<float, 3> velocity_mu{};
  dense(kLatentMu, encoded.data(), latent_mu.data());
  dense(kVelocityMu, encoded.data(), velocity_mu.data());
  for (float & value : latent_mu) {value = std::clamp(value, -10.0F, 10.0F);}
  for (float & value : velocity_mu) {value = std::clamp(value, -10.0F, 10.0F);}

  std::array<float, 64> actor_input{};
  std::copy(latent_mu.begin(), latent_mu.end(), actor_input.begin());
  std::copy(velocity_mu.begin(), velocity_mu.end(), actor_input.begin() + 16);
  // The deployment exporter uses the newest frame, not the complete history,
  // as the actor observation after the VAE posterior has been computed.
  std::copy(
    deployment_history.end() - kRlObservationSize, deployment_history.end(),
    actor_input.begin() + 19);

  std::array<float, 512> actor_hidden0{};
  std::array<float, 256> actor_hidden2{};
  std::array<float, 128> actor_hidden4{};
  dense(kActor0, actor_input.data(), actor_hidden0.data());
  applyElu(actor_hidden0);
  dense(kActor2, actor_hidden0.data(), actor_hidden2.data());
  applyElu(actor_hidden2);
  dense(kActor4, actor_hidden2.data(), actor_hidden4.data());
  applyElu(actor_hidden4);
  dense(kActor6, actor_hidden4.data(), action.data());
  if (dm1_policy_3610::kSquashActionMean) {
    for (float & value : action) {value = std::tanh(value);}
  }
  for (const float value : action) {
    if (!std::isfinite(value)) {return false;}
  }
  return true;
}

RlPolicyMetadata FrozenDwaqPolicy::metadata() const
{
  /*换模型要修改的地方：模型兼容槽位名；若修改需同步 FSM 校验*/
  return RlPolicyMetadata{
    // Keep the legacy contract name for the unchanged FSM selector; the
    // checkpoint hash above is the active model_3610 identity.
    "model_3285", kDm1FlatCheckpointSha256, false, true, true};
}
