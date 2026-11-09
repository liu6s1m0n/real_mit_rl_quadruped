//
// Created by qiayuan on 2022/7/1.
//

#pragma once

#include <array>

#include "legged_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

// Z = [广义加速度 ，接触力 ，关节力矩]
class WbcBase {
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

 public:
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  // 清空上一周期的期望输入（用于差分关节加速度），再次站立时不沿用上一次站立的数据。
  void reset() {
    inputLast_.setZero(info_.inputDim);
    contactAge_.fill(kNotInContact);
  }

  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period);

  // 触地检测覆盖，只对下一次 update() 生效：支撑腿按 contactFlag 而不是计划模式；
  // probing 的腿（计划支撑但还没真实触地）按摆腿处理，足端竖直期望速度为 -probeVelocity、不跟踪竖直位置。
  void setContactOverride(const contact_flag_t& contactFlag, const contact_flag_t& probing, scalar_t probeVelocity) {
    overrideFlag_ = contactFlag;
    overrideProbing_ = probing;
    overrideProbeVelocity_ = probeVelocity;
    overrideValid_ = true;
  }

 protected:
  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateNoContactMotionTask();
  Task formulateFrictionConeTask();
  Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  Task formulateSwingLegTask();
  Task formulateContactForceTask(const vector_t& inputDesired) const;
  // 落地力缓升：刚进入支撑的腿，竖直接触力不超过 比例×MPC 期望竖直力，比例在 contactRampDuration_ 内
  // 从 contactRampInitialRatio_ 线性升到 1；时长为 0 时不加约束。
  Task formulateContactForceRampTask() const;

  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  vector_t qMeasured_, vMeasured_, inputLast_;
  matrix_t j_, dj_;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // 落地力缓升状态：各接触点进入支撑后的时长 [s]，未支撑时为 kNotInContact；
  // inputDesiredLast_ 保存本周期 MPC 期望输入，用于取期望竖直力。
  static constexpr scalar_t kNotInContact = -1.0;
  std::array<scalar_t, 4> contactAge_{{kNotInContact, kNotInContact, kNotInContact, kNotInContact}};
  vector_t inputDesiredLast_;

  // 触地检测覆盖（setContactOverride），probing_ 为本周期正在下探的腿
  bool overrideValid_ = false;
  contact_flag_t overrideFlag_{}, overrideProbing_{}, probing_{};
  scalar_t overrideProbeVelocity_ = 0.0, probeVelocity_ = 0.0;

  // Task Parameters:
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
  scalar_t contactRampDuration_{0.0}, contactRampInitialRatio_{1.0};
};

}  // namespace legged
