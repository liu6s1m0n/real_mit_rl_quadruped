#include <gtest/gtest.h>

// 验证 MPC 步态表按工程约定的 FR、FL、RR、RL 顺序生成小跑接触预测。

#include "MPC/Gait.h"
#include "MPC/SolverMPC.h"
#include "controller/GaitScheduler.hpp"

namespace
{

constexpr std::size_t kMpcHorizon = mpc::kDefaultMpcHorizon;
constexpr std::size_t kTrotHalfHorizon = kMpcHorizon / 2;

TEST(MpcGait, BuildsTrotContactPredictionInProjectLegOrder)
{
  // 10 个预测段、半周期相位差和 5 段支撑共同构成 50% 支撑 TROT。
  mpc::OffsetDurationGait gait(
    kMpcHorizon,
    {0, kTrotHalfHorizon, kTrotHalfHorizon, 0},
    {kTrotHalfHorizon, kTrotHalfHorizon, kTrotHalfHorizon, kTrotHalfHorizon},
    "trot");
  gait.advance(0, kMpcHorizon);
  const auto & table = gait.contactTable();
  ASSERT_EQ(table.size(), kMpcHorizon * kNumLegs);
  // 两组对角腿严格交替。
  EXPECT_EQ(table[0], 1);
  EXPECT_EQ(table[1], 0);
  EXPECT_EQ(table[2], 0);
  EXPECT_EQ(table[3], 1);
  const std::size_t firstSwingRow = kTrotHalfHorizon + 1;
  EXPECT_EQ(table[firstSwingRow * kNumLegs], 0);
  EXPECT_EQ(table[firstSwingRow * kNumLegs + 1], 1);
}

TEST(MpcGait, FirstPredictionRowUsesMitNextSegment)
{
  mpc::OffsetDurationGait gait(
    kMpcHorizon,
    {0, kTrotHalfHorizon, kTrotHalfHorizon, 0},
    {kTrotHalfHorizon, kTrotHalfHorizon, kTrotHalfHorizon, kTrotHalfHorizon},
    "trot");
  // Segment 4 is the last segment of the FR/RL stance half-cycle.  MIT's
  // getMpcTable() starts row zero at segment 5, so it must already switch.
  gait.advance(kTrotHalfHorizon - 1, 1);
  const auto & table = gait.contactTable();
  EXPECT_EQ(table[0], 0);
  EXPECT_EQ(table[1], 1);
  EXPECT_EQ(table[2], 1);
  EXPECT_EQ(table[3], 0);
}

TEST(MpcGait, SchedulerFrameCarriesCurrentContactAndMitPrediction)
{
  GaitScheduler<float> scheduler(0.001F);
  scheduler.requestGait(GaitType::TROT);
  scheduler.step();
  const auto frame = scheduler.gaitFrame(kMpcHorizon);

  ASSERT_TRUE(frame.valid(kMpcHorizon));
  EXPECT_EQ(frame.contact_state,
    (std::array<bool, kNumLegs>{true, false, false, true}));
  EXPECT_EQ(frame.mpc_table[0], 1);
  EXPECT_EQ(frame.mpc_table[1], 0);
  EXPECT_EQ(frame.mpc_table[2], 0);
  EXPECT_EQ(frame.mpc_table[3], 1);
  EXPECT_GT(frame.contact_probability[0], 0.0F);
  EXPECT_LT(frame.contact_probability[0], 1.0F);
  EXPECT_FLOAT_EQ(frame.contact_probability[1], 0.0F);
}

TEST(MpcGait, SchedulerFrameUsesNextSegmentAtBoundary)
{
  GaitScheduler<float> scheduler(0.001F);
  scheduler.requestGait(GaitType::TROT);

  // Advance into segment 4 (the last FR/RL support segment) without relying
  // on a hard-coded control-frame count. The row-zero prediction must already
  // describe segment 5, as in MIT's getMpcTable().
  bool reached_segment = false;
  for (std::size_t step = 0; step < 2 * kMpcHorizon * 30; ++step) {
    scheduler.step();
    const float phase = scheduler.gait_data.phase_variable(0);
    if (phase >= 0.4F && phase < 0.5F) {
      reached_segment = true;
      break;
    }
  }
  ASSERT_TRUE(reached_segment);

  const auto frame = scheduler.gaitFrame(kMpcHorizon);
  EXPECT_TRUE(frame.contact_state[0]);
  EXPECT_FALSE(frame.contact_state[1]);
  EXPECT_FALSE(frame.contact_state[2]);
  EXPECT_TRUE(frame.contact_state[3]);
  EXPECT_EQ(frame.mpc_table[0], 0);
  EXPECT_EQ(frame.mpc_table[1], 1);
  EXPECT_EQ(frame.mpc_table[2], 1);
  EXPECT_EQ(frame.mpc_table[3], 0);
}

}  // namespace
