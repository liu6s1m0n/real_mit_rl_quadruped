/**
 * @file GaitFrame.hpp
 * @brief Immutable per-cycle gait data shared by estimation and control.
 */
#ifndef MYMIT_ROBOT_CONTROLLER_GAIT_FRAME_HPP_
#define MYMIT_ROBOT_CONTROLLER_GAIT_FRAME_HPP_

#include <array>
#include <cstddef>
#include <vector>

#include "model/robot_types.hpp"

/** A single control-cycle snapshot of the active gait. */
template<typename T>
struct GaitFrame
{
  GaitFrame()
  {
    contact_state.fill(true);
    contact_probability.fill(T(1));
    mpc_table.assign(10 * kNumLegs, 1);
  }

  /** 二值接触拓扑：true 时加入 MPC/WBC 支撑接触约束。 */
  std::array<bool, kNumLegs> contact_state{};
  /** MIT 风格的连续支撑相位，供状态估计器使用，范围为 [0, 1]。 */
  std::array<T, kNumLegs> contact_probability{};
  std::array<T, kNumLegs> swing_phase{};
  std::array<T, kNumLegs> stance_time{};
  std::array<T, kNumLegs> swing_time{};
  /** MIT convention: row zero is the segment after the current segment. */
  std::vector<int> mpc_table;

  bool valid(std::size_t horizon) const noexcept
  {
    return horizon > 0 && mpc_table.size() == horizon * kNumLegs;
  }
};

#endif  // MYMIT_ROBOT_CONTROLLER_GAIT_FRAME_HPP_
