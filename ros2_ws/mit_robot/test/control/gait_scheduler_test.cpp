#include <array>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "controller/GaitScheduler.hpp"

namespace
{

constexpr float kTolerance = 1e-5F;

}  // namespace

TEST(GaitSchedulerTest, InitializesAllLegsInStandingContact)
{
  GaitScheduler<float> scheduler(0.002F);

  EXPECT_EQ(scheduler.gait_data.current_gait, GaitType::STAND);
  EXPECT_EQ(scheduler.gait_data.gait_name, "STAND");
  scheduler.step();

  for (std::size_t index = 0; index < kNumLegs; ++index) {
    const LegId leg = static_cast<LegId>(index);
    EXPECT_TRUE(scheduler.gait_data.contactScheduled(leg));
    EXPECT_EQ(
      scheduler.gait_data.touchdown_scheduled(
        static_cast<Eigen::Index>(index)), 1);
  }

  scheduler.step();
  EXPECT_TRUE(scheduler.gait_data.touchdown_scheduled.isZero());
  EXPECT_TRUE(scheduler.gait_data.liftoff_scheduled.isZero());
}

TEST(GaitSchedulerTest, PreservesTrotDiagonalPhasePattern)
{
  GaitScheduler<float> scheduler(0.001F);
  scheduler.requestGait(GaitType::TROT);
  scheduler.step();

  EXPECT_EQ(scheduler.gait_data.current_gait, GaitType::TROT);
  // DM1 TROT：10 段 x 30 ms，与 MPC 接触表同周期。
  EXPECT_NEAR(scheduler.gait_data.period_time_nominal, 0.30F, kTolerance);
  EXPECT_NEAR(scheduler.gait_data.switching_phase_nominal, 0.5F, kTolerance);

  // 本工程腿序为 FR、FL、RR、RL；对角腿同相。
  // 50%支撑：两组对角腿交替，无额外支撑重叠。
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(0), 1);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(1), 0);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(2), 0);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(3), 1);
  EXPECT_NEAR(
    scheduler.gait_data.phase_variable(0),
    scheduler.gait_data.phase_variable(3), kTolerance);
  EXPECT_NEAR(
    scheduler.gait_data.phase_variable(1),
    scheduler.gait_data.phase_variable(2), kTolerance);

  const auto probabilities =
    scheduler.gait_data.scheduledContactProbabilities();
  EXPECT_GT(probabilities[0], 0.0F);
  EXPECT_GT(probabilities[3], 0.0F);
  EXPECT_FLOAT_EQ(probabilities[1], 0.0F);
  EXPECT_FLOAT_EQ(probabilities[2], 0.0F);
}

TEST(GaitSchedulerTest, TwoLegStandKeepsFrRlAndLiftsFlRr)
{
  GaitScheduler<float> scheduler(0.002F);
  scheduler.requestGait(GaitType::TWO_LEG_STAND);
  scheduler.step();

  EXPECT_EQ(scheduler.gait_data.current_gait, GaitType::TWO_LEG_STAND);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(0), 1);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(1), 0);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(2), 0);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(3), 1);
  EXPECT_TRUE(scheduler.gait_data.time_swing(1) > 0.0F);
  EXPECT_TRUE(scheduler.gait_data.time_swing(2) > 0.0F);
  EXPECT_FLOAT_EQ(scheduler.gait_data.time_swing(0), 0.0F);
  EXPECT_FLOAT_EQ(scheduler.gait_data.time_swing(3), 0.0F);

  constexpr std::size_t horizon = 10;
  const auto frame = scheduler.gaitFrame(horizon);
  for (std::size_t step = 0; step < horizon; ++step) {
    EXPECT_EQ(frame.mpc_table[step * kNumLegs + 0], 1);
    EXPECT_EQ(frame.mpc_table[step * kNumLegs + 1], step < horizon / 2 - 1 ? 0 : 1);
    EXPECT_EQ(frame.mpc_table[step * kNumLegs + 2], step < horizon / 2 - 1 ? 0 : 1);
    EXPECT_EQ(frame.mpc_table[step * kNumLegs + 3], 1);
  }
}

TEST(GaitSchedulerTest, TwoLegStandReverseKeepsFlRrAndLiftsFrRl)
{
  GaitScheduler<float> scheduler(0.002F);
  scheduler.requestGait(GaitType::TWO_LEG_STAND_REVERSE);
  scheduler.step();

  EXPECT_EQ(scheduler.gait_data.current_gait, GaitType::TWO_LEG_STAND_REVERSE);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(0), 0);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(1), 1);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(2), 1);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(3), 0);
  EXPECT_FLOAT_EQ(scheduler.gait_data.time_swing(0), scheduler.gait_data.period_time(0) / 2);
  EXPECT_FLOAT_EQ(scheduler.gait_data.time_swing(3), scheduler.gait_data.period_time(3) / 2);
  EXPECT_FLOAT_EQ(scheduler.gait_data.time_swing(1), 0.0F);
  EXPECT_FLOAT_EQ(scheduler.gait_data.time_swing(2), 0.0F);
}

TEST(GaitSchedulerTest, SingleDiagonalStepsLandOnceAndNeverWrap)
{
  for (const auto gait : {GaitType::TWO_LEG_STAND, GaitType::TWO_LEG_STAND_REVERSE}) {
    SCOPED_TRACE(static_cast<int>(gait));
    GaitScheduler<float> scheduler(0.002F);
    scheduler.step();  // 从完整支撑开始，统计真实离地和触地沿。
    scheduler.requestGait(gait);
    std::array<int, kNumLegs> liftoffs{}, touchdowns{};
    int landing_step = -1;
    for (int step = 0; step < 600; ++step) {
      scheduler.step();
      const auto & data = scheduler.gait_data;
      const auto frame = scheduler.gaitFrame(10);
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        liftoffs[leg] += data.liftoff_scheduled(leg);
        touchdowns[leg] += data.touchdown_scheduled(leg);
        EXPECT_GE(data.time_stance_remaining(leg), 0.0F);
        EXPECT_GE(data.time_swing_remaining(leg), 0.0F);
        // MPC 在预测到落地后，不能再安排这条腿离地。
        bool predicted_landed = false;
        for (std::size_t row = 0; row < 10; ++row) {
          const bool contact = frame.mpc_table[row * kNumLegs + leg] != 0;
          if (predicted_landed) {EXPECT_TRUE(contact);}
          predicted_landed = predicted_landed || contact;
        }
      }
      if (data.initial_phase >= 1.0F) {
        if (landing_step < 0) {landing_step = step;}
        EXPECT_TRUE(data.contact_state_scheduled.isOnes());
        EXPECT_TRUE(data.phase_swing.isZero());
        for (const auto p : frame.contact_probability) {EXPECT_FLOAT_EQ(p, 1.0F);}
        for (const int contact : frame.mpc_table) {EXPECT_EQ(contact, 1);}
      }
    }
    EXPECT_NEAR((landing_step + 1) * 0.002, 0.15, 0.003);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const bool swing = gait == GaitType::TWO_LEG_STAND ?
        (leg == 1 || leg == 2) : (leg == 0 || leg == 3);
      EXPECT_EQ(liftoffs[leg], swing ? 1 : 0);
      EXPECT_EQ(touchdowns[leg], swing ? 1 : 0);
    }
  }
}

TEST(GaitSchedulerTest, EstimatorContactPhaseMatchesMitProgress)
{
  GaitScheduler<float> scheduler(0.002F);
  EXPECT_EQ(
    scheduler.gait_data.estimatorContactProbabilities(),
    (std::array<float, kNumLegs>{1.0F, 1.0F, 1.0F, 1.0F}));

  scheduler.requestGait(GaitType::TROT);
  scheduler.step();
  const auto touchdown = scheduler.gait_data.estimatorContactProbabilities();
  EXPECT_GT(touchdown[0], 0.0F);
  EXPECT_GT(touchdown[3], 0.0F);
  EXPECT_FLOAT_EQ(touchdown[1], 0.0F);
  EXPECT_FLOAT_EQ(touchdown[2], 0.0F);

  // MIT contactState 是支撑相位进度；32 帧对应 0.064/0.30/0.5。
  for (int step = 0; step < 31; ++step) {scheduler.step();}
  const auto mid_stance = scheduler.gait_data.estimatorContactProbabilities();
  EXPECT_NEAR(mid_stance[0], 0.4266667F, 1e-5F);
  EXPECT_NEAR(mid_stance[3], 0.4266667F, 1e-5F);
}

TEST(GaitSchedulerTest, AppliesRuntimeTimingOverrideToOverrideableGait)
{
  GaitSchedulerParameters<float> parameters;
  parameters.override_mode = GaitOverrideMode::OVERRIDE_TIMING;
  parameters.gait_type = GaitType::TROT;
  parameters.gait_period_time = 0.8F;
  parameters.gait_switching_phase = 0.6F;
  GaitScheduler<float> scheduler(parameters, 0.002F);

  scheduler.step();

  EXPECT_EQ(scheduler.gait_data.current_gait, GaitType::TROT);
  EXPECT_NEAR(scheduler.gait_data.period_time_nominal, 0.8F, kTolerance);
  EXPECT_NEAR(scheduler.gait_data.switching_phase_nominal, 0.6F, kTolerance);
  EXPECT_TRUE(
    scheduler.gait_data.time_stance.isApprox(
      Vec4<float>::Constant(0.48F), kTolerance));
  EXPECT_TRUE(
    scheduler.gait_data.time_swing.isApprox(
      Vec4<float>::Constant(0.32F), kTolerance));
}

TEST(GaitSchedulerTest, RejectsInvalidParametersAndTimeStep)
{
  EXPECT_THROW(GaitScheduler<float>(0.0F), std::invalid_argument);

  GaitSchedulerParameters<float> parameters;
  parameters.gait_period_time =
    std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(GaitScheduler<float>(parameters, 0.002F), std::invalid_argument);
}
