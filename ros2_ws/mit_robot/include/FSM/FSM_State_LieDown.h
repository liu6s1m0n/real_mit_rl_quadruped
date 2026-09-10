/**
 * @file FSM_State_LieDown.h
 * @brief MPC/WBC supported descent followed by a safe grounded fold.
 */
#ifndef MYMIT_ROBOT_FSM_STATE_LIE_DOWN_H_
#define MYMIT_ROBOT_FSM_STATE_LIE_DOWN_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "FSM/FSM_State.h"
#include "MPC/ConvexMPCLocomotion.h"
#include "WBC/LocomotionCtrl/LocomotionCtrl.hpp"

/**
 * @brief High-level prone transition.
 *
 * The lowering phase uses the existing MPC and WBC implementations with the
 * existing four-foot STAND contact model. Once the body is close to the
 * grounded home pose, the state performs a smooth joint fold and then keeps
 * the existing MPC/WBC contact model active for a grounded hold.
 * After folding, the state remains active in a grounded MPC/WBC hold so the
 * floating base is damped; it does not hand off to joint-only control.
 * This state deliberately has no RlPolicy dependency.
 */
template<typename T>
class FSM_State_LieDown final : public FSM_State<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit FSM_State_LieDown(ControlFSMData<T> * control_fsm_data);
  ~FSM_State_LieDown() override = default;

  void onEnter() override;
  void run() override;
  /** @brief Folding 完成后仍保持 MPC/WBC 机身阻尼。 */
  bool isComplete() const noexcept
  {return phase_ == Phase::Hold && loweringPostureStable();}
  FSM_StateName checkTransition() override;
  TransitionData<T> transition() override;
  void onExit() override;

private:
  enum class Phase : std::uint8_t
  {
    Lowering,
    FoldingCalf,
    FoldingThighAndCalf,
    Hold
  };

  void runMpcLowering();
  void runJointFolding();
  void runProneHold();
  bool loweringPostureStable() const noexcept;
  std::array<Vec3<T>, kNumLegs> footPositionsWorld() const;

  std::unique_ptr<mpc::ConvexMPCLocomotion<T>> mpc_;
  std::unique_ptr<LocomotionCtrl<T>> wbc_ctrl_;
  LocomotionCtrlData<T> wbc_data_;
  std::array<Vec3<T>, kNumLegs> entry_joint_positions_{};
  // Folding starts from the posture actually reached by MPC/WBC. Using the
  // LieDown entry posture here would create a large position step at the
  // Lowering -> FoldingCalf handoff.
  std::array<Vec3<T>, kNumLegs> fold_start_joint_positions_{};
  Vec3<T> hold_body_position_world_ = Vec3<T>::Zero();
  Phase phase_ = Phase::Lowering;
  std::size_t fold_iteration_ = 0;
  std::size_t fold_iterations_ = 1;
  std::size_t calf_fold_iterations_ = 1;
  std::size_t thigh_fold_iterations_ = 1;
  T prone_body_height_ = T(0.12);
};

extern template class FSM_State_LieDown<float>;

#endif  // MYMIT_ROBOT_FSM_STATE_LIE_DOWN_H_
