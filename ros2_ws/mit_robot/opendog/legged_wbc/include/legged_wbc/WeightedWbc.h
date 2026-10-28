//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WbcBase.h"

namespace legged {

class WeightedWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;

  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

  // Diagnostic return codes only; the original QP/output path is unchanged.
  int lastSolveStatus() const { return lastSolveStatus_; }
  int lastSolutionStatus() const { return lastSolutionStatus_; }

 protected:
  virtual Task formulateConstraints();
  virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);

 private:
  scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
  int lastSolveStatus_ = 0;
  int lastSolutionStatus_ = 0;
};

}  // namespace legged
