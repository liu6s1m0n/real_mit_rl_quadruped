/*! @file contact_detector_test.cpp
 *  @brief ContactDetector 状态机：计划摆动、力/高度触发、连续确认和最长等待。
 */
#include <gtest/gtest.h>

#include "control/contact_detector.hpp"

namespace
{
std::array<ContactSignal, ContactDetector::kLegs> airborne(bool planned_stance)
{
  std::array<ContactSignal, ContactDetector::kLegs> signals{};
  for (auto & signal : signals) {
    signal.planned_stance = planned_stance;
    signal.force_z = 10.0;
    signal.height = 0.02;
    signal.velocity_z = -0.3;
  }
  return signals;
}
}  // namespace

TEST(ContactDetector, SwingLegsAreNeverInContact)
{
  ContactDetector detector;
  auto signals = airborne(false);
  signals[0].force_z = 100.0;
  signals[0].height = 0.0;
  signals[0].velocity_z = 0.0;
  for (int tick = 0; tick < 10; ++tick) {detector.update(signals, 0.002);}
  for (bool contact : detector.detected()) {EXPECT_FALSE(contact);}
}

TEST(ContactDetector, ForceOrLandingNeedsConfirmTicks)
{
  ContactDetectionSettings settings;
  settings.confirm_ticks = 2;
  ContactDetector detector(settings);
  auto signals = airborne(true);
  signals[0].force_z = 60.0;           // 力超过阈值
  signals[1].height = 0.002;           // 接近地面且停止下降
  signals[1].velocity_z = -0.01;
  signals[2].height = 0.002;           // 接近地面但仍在快速下降：不算
  detector.update(signals, 0.002);
  EXPECT_FALSE(detector.detected()[0]);
  EXPECT_FALSE(detector.detected()[1]);
  detector.update(signals, 0.002);
  EXPECT_TRUE(detector.detected()[0]);
  EXPECT_TRUE(detector.detected()[1]);
  EXPECT_FALSE(detector.detected()[2]);
  EXPECT_TRUE(detector.waiting(2, true));
  // 已触地后条件消失也保持，直到计划摆动
  detector.update(airborne(true), 0.002);
  EXPECT_TRUE(detector.detected()[0]);
  detector.update(airborne(false), 0.002);
  EXPECT_FALSE(detector.detected()[0]);
}

TEST(ContactDetector, InterruptedConditionRestartsConfirmation)
{
  ContactDetectionSettings settings;
  settings.confirm_ticks = 3;
  ContactDetector detector(settings);
  auto loaded = airborne(true);
  loaded[0].force_z = 60.0;
  detector.update(loaded, 0.002);
  detector.update(loaded, 0.002);
  detector.update(airborne(true), 0.002);
  detector.update(loaded, 0.002);
  detector.update(loaded, 0.002);
  EXPECT_FALSE(detector.detected()[0]);
  detector.update(loaded, 0.002);
  EXPECT_TRUE(detector.detected()[0]);
}

TEST(ContactDetector, MaxDelayForcesContact)
{
  ContactDetectionSettings settings;
  settings.max_delay = 0.01;
  ContactDetector detector(settings);
  for (int tick = 0; tick < 4; ++tick) {detector.update(airborne(true), 0.002);}
  EXPECT_FALSE(detector.detected()[3]);
  detector.update(airborne(true), 0.002);
  EXPECT_TRUE(detector.detected()[3]);
}

TEST(ContactDetector, AcceptPlannedAndReset)
{
  ContactDetector detector;
  detector.acceptPlanned({{true, false, true, false}});
  EXPECT_TRUE(detector.detected()[0]);
  EXPECT_FALSE(detector.detected()[1]);
  detector.reset();
  for (bool contact : detector.detected()) {EXPECT_FALSE(contact);}
}

TEST(ContactDetector, DisabledLegsFollowPlanAndZeroHeightDisablesKinematics)
{
  ContactDetectionSettings settings;
  settings.enabled = {{false, false, true, true}};
  settings.height_threshold = 0.0;
  ContactDetector detector(settings);
  auto signals = airborne(true);
  signals[2].height = -0.01;
  signals[2].velocity_z = 0.0;
  detector.update(signals, 0.002);
  detector.update(signals, 0.002);
  EXPECT_TRUE(detector.detected()[0]);
  EXPECT_TRUE(detector.detected()[1]);
  EXPECT_FALSE(detector.detected()[2]);
  signals[2].force_z = 60.0;
  detector.update(signals, 0.002);
  detector.update(signals, 0.002);
  EXPECT_TRUE(detector.detected()[2]);
  detector.update(airborne(false), 0.002);
  EXPECT_FALSE(detector.detected()[0]);
}
