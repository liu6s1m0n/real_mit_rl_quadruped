#include "controller/frozen_dwaq_policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "dm1_policy_4210.hpp"
#include "dm1_policy_4245.hpp"

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

struct DwaqWeights
{
  DenseLayer encoder0;
  DenseLayer encoder2;
  DenseLayer latent_mu;
  DenseLayer velocity_mu;
  DenseLayer actor0;
  DenseLayer actor2;
  DenseLayer actor4;
  DenseLayer actor6;
  const char * model_name;
  const char * checkpoint_sha256;
  bool squash_action_mean;
};

const DwaqWeights kModel4210Weights{
  {
    dm1_policy_4210::k_vae_encoder_encoder_0_weight.data(),
    dm1_policy_4210::k_vae_encoder_encoder_0_bias.data(), 128, 270},
  {
    dm1_policy_4210::k_vae_encoder_encoder_2_weight.data(),
    dm1_policy_4210::k_vae_encoder_encoder_2_bias.data(), 64, 128},
  {
    dm1_policy_4210::k_vae_latent_mu_weight.data(),
    dm1_policy_4210::k_vae_latent_mu_bias.data(), 16, 64},
  {
    dm1_policy_4210::k_vae_vel_mu_weight.data(),
    dm1_policy_4210::k_vae_vel_mu_bias.data(), 3, 64},
  {
    dm1_policy_4210::k_actor_0_weight.data(),
    dm1_policy_4210::k_actor_0_bias.data(), 512, 64},
  {
    dm1_policy_4210::k_actor_2_weight.data(),
    dm1_policy_4210::k_actor_2_bias.data(), 256, 512},
  {
    dm1_policy_4210::k_actor_4_weight.data(),
    dm1_policy_4210::k_actor_4_bias.data(), 128, 256},
  {
    dm1_policy_4210::k_actor_6_weight.data(),
    dm1_policy_4210::k_actor_6_bias.data(), 12, 128},
  "model_4210", kDm1FlatCheckpointSha256, dm1_policy_4210::kSquashActionMean};

const DwaqWeights kModel4245Weights{
  {
    dm1_policy_4245::k_vae_encoder_encoder_0_weight.data(),
    dm1_policy_4245::k_vae_encoder_encoder_0_bias.data(), 128, 270},
  {
    dm1_policy_4245::k_vae_encoder_encoder_2_weight.data(),
    dm1_policy_4245::k_vae_encoder_encoder_2_bias.data(), 64, 128},
  {
    dm1_policy_4245::k_vae_latent_mu_weight.data(),
    dm1_policy_4245::k_vae_latent_mu_bias.data(), 16, 64},
  {
    dm1_policy_4245::k_vae_vel_mu_weight.data(),
    dm1_policy_4245::k_vae_vel_mu_bias.data(), 3, 64},
  {
    dm1_policy_4245::k_actor_0_weight.data(),
    dm1_policy_4245::k_actor_0_bias.data(), 512, 64},
  {
    dm1_policy_4245::k_actor_2_weight.data(),
    dm1_policy_4245::k_actor_2_bias.data(), 256, 512},
  {
    dm1_policy_4245::k_actor_4_weight.data(),
    dm1_policy_4245::k_actor_4_bias.data(), 128, 256},
  {
    dm1_policy_4245::k_actor_6_weight.data(),
    dm1_policy_4245::k_actor_6_bias.data(), 12, 128},
  "model_4245", kDm1YawRecoveryCheckpointSha256, dm1_policy_4245::kSquashActionMean};

const DwaqWeights & weightsFor(FrozenDwaqModel model)
{
  return model == FrozenDwaqModel::Model4245 ? kModel4245Weights : kModel4210Weights;
}
}  // namespace

bool FrozenDwaqPolicy::infer(
  const std::array<float, kRlObservationSize> & observation,
  const std::array<float, kRlObservationSize * kRlHistoryLength> & history,
  std::array<float, kRlActionSize> & action)
{
  const DwaqWeights & weights = weightsFor(model_);
  for (const float value : observation) {
    if (!std::isfinite(value) || std::abs(value) > 100.0F) {return false;}
  }
  for (const float value : history) {
    if (!std::isfinite(value) || std::abs(value) > 100.0F) {return false;}
  }
  // DM1 training initializes obs_hist_buf by repeating the first observation
  // across all six frames. Keep this defensive compatibility path for callers
  // that have not initialized the deployment history yet; the FSM normally
  // performs this initialization explicitly before the first RL inference.
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
  dense(weights.encoder0, deployment_history.data(), encoder_hidden.data());
  applyElu(encoder_hidden);
  dense(weights.encoder2, encoder_hidden.data(), encoded.data());

  std::array<float, 16> latent_mu{};
  std::array<float, 3> velocity_mu{};
  dense(weights.latent_mu, encoded.data(), latent_mu.data());
  dense(weights.velocity_mu, encoded.data(), velocity_mu.data());
  for (float & value : latent_mu) {value = std::clamp(value, -10.0F, 10.0F);}
  for (float & value : velocity_mu) {value = std::clamp(value, -10.0F, 10.0F);}

  std::array<float, 64> actor_input{};
  std::copy(latent_mu.begin(), latent_mu.end(), actor_input.begin());
  std::copy(velocity_mu.begin(), velocity_mu.end(), actor_input.begin() + 16);
  // Training uses the current single-frame observation for the actor. The
  // history above is only the VAE input and ends at the previous policy step.
  // Do not use the history's last frame here: doing so creates a one-frame
  // temporal skew between the actor and the observation used by training.
  std::copy(observation.begin(), observation.end(), actor_input.begin() + 19);

  std::array<float, 512> actor_hidden0{};
  std::array<float, 256> actor_hidden2{};
  std::array<float, 128> actor_hidden4{};
  dense(weights.actor0, actor_input.data(), actor_hidden0.data());
  applyElu(actor_hidden0);
  dense(weights.actor2, actor_hidden0.data(), actor_hidden2.data());
  applyElu(actor_hidden2);
  dense(weights.actor4, actor_hidden2.data(), actor_hidden4.data());
  applyElu(actor_hidden4);
  dense(weights.actor6, actor_hidden4.data(), action.data());
  if (weights.squash_action_mean) {
    for (float & value : action) {value = std::tanh(value);}
  }
  for (const float value : action) {
    if (!std::isfinite(value)) {return false;}
  }
  return true;
}

RlPolicyMetadata FrozenDwaqPolicy::metadata() const
{
  return RlPolicyMetadata{
    weightsFor(model_).model_name, weightsFor(model_).checkpoint_sha256,
    false, true, true};
}
