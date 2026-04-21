// Unit tests for StallBreaker helper.

#include "../src/stall_breaker.h"

#include <gtest/gtest.h>

namespace {

StallBreakerConfig defaultCfg() {
  StallBreakerConfig c;
  c.enabled = true;
  c.min_cmd_duty = 0.05;
  c.rpm_threshold = 50;
  c.detect_time_ms = 300;
  c.pulse_duty = 0.8;
  c.pulse_time_ms = 500;
  c.cooldown_time_ms = 1000;
  c.max_consecutive_pulses = 5;
  c.motion_reset_ms = 200;
  return c;
}

// Step a StallBreaker forward by `total_ms`, calling update every `step_ms` with
// the given command and rpm. Returns the last duty observed.
double run(StallBreaker& sb, double cmd, int rpm, ros::Time& t, int total_ms, int step_ms = 20) {
  double out = cmd;
  for (int elapsed = 0; elapsed < total_ms; elapsed += step_ms) {
    t += ros::Duration(step_ms / 1000.0);
    out = sb.update(cmd, rpm, t);
  }
  return out;
}

}  // namespace

TEST(StallBreaker, DisabledPassesThrough) {
  StallBreaker sb("L");
  StallBreakerConfig c = defaultCfg();
  c.enabled = false;
  sb.setConfig(c);
  ros::Time t(1000.0);
  // Even a long stalled-command stream returns cmd unchanged.
  const double out = run(sb, 0.2, 0, t, 2000);
  EXPECT_DOUBLE_EQ(out, 0.2);
  EXPECT_EQ(sb.state(), StallBreaker::State::IDLE);
  EXPECT_EQ(sb.consecutive_pulses(), 0);
}

TEST(StallBreaker, NoCommandNoPulse) {
  StallBreaker sb("L");
  sb.setConfig(defaultCfg());
  ros::Time t(1000.0);
  const double out = run(sb, 0.0, 0, t, 2000);
  EXPECT_DOUBLE_EQ(out, 0.0);
  EXPECT_EQ(sb.state(), StallBreaker::State::IDLE);
}

TEST(StallBreaker, MovingFreelyNoPulse) {
  StallBreaker sb("L");
  sb.setConfig(defaultCfg());
  ros::Time t(1000.0);
  const double out = run(sb, 0.3, 200, t, 2000);
  EXPECT_DOUBLE_EQ(out, 0.3);
  EXPECT_EQ(sb.state(), StallBreaker::State::IDLE);
}

TEST(StallBreaker, StalledTriggersPulseAfterDwell) {
  StallBreaker sb("L");
  sb.setConfig(defaultCfg());
  ros::Time t(1000.0);

  // Before dwell elapses (200 ms of 300 ms), should pass through.
  for (int i = 0; i < 10; ++i) {  // 10 * 20 ms = 200 ms
    t += ros::Duration(0.02);
    EXPECT_DOUBLE_EQ(sb.update(0.2, 0, t), 0.2);
  }
  EXPECT_EQ(sb.state(), StallBreaker::State::DETECTING);

  // Step past 300 ms dwell — should transition to PULSING and return boosted duty.
  for (int i = 0; i < 10; ++i) {  // another 200 ms (total 400 ms from first)
    t += ros::Duration(0.02);
    double out = sb.update(0.2, 0, t);
    if (sb.state() == StallBreaker::State::PULSING) {
      EXPECT_DOUBLE_EQ(out, 0.8);
      return;
    }
  }
  FAIL() << "Never entered PULSING after dwell";
}

TEST(StallBreaker, PulseEndsAfterDuration) {
  StallBreaker sb("L");
  sb.setConfig(defaultCfg());
  ros::Time t(1000.0);
  // Drive to PULSING
  run(sb, 0.2, 0, t, 400);
  ASSERT_EQ(sb.state(), StallBreaker::State::PULSING);
  // Step through the pulse (500 ms)
  run(sb, 0.2, 0, t, 520);
  EXPECT_EQ(sb.state(), StallBreaker::State::COOLDOWN);
  // During cooldown, duty is passed through even though still stalled.
  t += ros::Duration(0.02);
  EXPECT_DOUBLE_EQ(sb.update(0.2, 0, t), 0.2);
}

TEST(StallBreaker, CooldownPreventsImmediateRepulse) {
  StallBreaker sb("L");
  sb.setConfig(defaultCfg());
  ros::Time t(1000.0);
  // Pulse once
  run(sb, 0.2, 0, t, 400);
  ASSERT_EQ(sb.state(), StallBreaker::State::PULSING);
  run(sb, 0.2, 0, t, 520);
  ASSERT_EQ(sb.state(), StallBreaker::State::COOLDOWN);
  ASSERT_EQ(sb.consecutive_pulses(), 1);
  // While in cooldown (for <1000 ms) no pulse occurs even though still stalled.
  for (int elapsed = 0; elapsed < 900; elapsed += 20) {
    t += ros::Duration(0.02);
    double out = sb.update(0.2, 0, t);
    EXPECT_DOUBLE_EQ(out, 0.2);
    EXPECT_NE(sb.state(), StallBreaker::State::PULSING);
  }
  EXPECT_EQ(sb.consecutive_pulses(), 1);  // still just one pulse so far
  // After cooldown + another 300 ms dwell, a second pulse should fire.
  run(sb, 0.2, 0, t, 500);  // finish cooldown + traverse DETECTING dwell
  EXPECT_EQ(sb.state(), StallBreaker::State::PULSING);
  EXPECT_EQ(sb.consecutive_pulses(), 2);
}

TEST(StallBreaker, MotionResetsCounter) {
  StallBreaker sb("L");
  sb.setConfig(defaultCfg());
  ros::Time t(1000.0);
  // Force 3 consecutive pulses. A full cycle is dwell(300)+pulse(500)+cooldown(1000)
  // ≈ 1800 ms. Drive long enough for 3 cycles, then stop.
  run(sb, 0.2, 0, t, 1800 * 3);
  ASSERT_GE(sb.consecutive_pulses(), 3);
  // Now sustain motion for >= motion_reset_ms — counter should reset.
  run(sb, 0.3, 200, t, 260);
  EXPECT_EQ(sb.consecutive_pulses(), 0);
}

TEST(StallBreaker, UnrecoverableAfterNConsecutive) {
  StallBreaker sb("L");
  StallBreakerConfig c = defaultCfg();
  c.max_consecutive_pulses = 2;  // reduce to keep test short
  sb.setConfig(c);
  ros::Time t(1000.0);
  // Pulse 1
  run(sb, 0.2, 0, t, 400);
  ASSERT_EQ(sb.state(), StallBreaker::State::PULSING);
  run(sb, 0.2, 0, t, 520);
  run(sb, 0.2, 0, t, 1020);
  // Pulse 2
  run(sb, 0.2, 0, t, 400);
  ASSERT_EQ(sb.state(), StallBreaker::State::PULSING);
  EXPECT_EQ(sb.consecutive_pulses(), 2);
  run(sb, 0.2, 0, t, 520);
  run(sb, 0.2, 0, t, 1020);
  // Next attempt: after dwell, should transition to UNRECOVERABLE, not PULSING.
  run(sb, 0.2, 0, t, 400);
  EXPECT_EQ(sb.state(), StallBreaker::State::UNRECOVERABLE);
  // Output stays at commanded (no pulse).
  t += ros::Duration(0.02);
  EXPECT_DOUBLE_EQ(sb.update(0.2, 0, t), 0.2);
  // Sustained motion recovers out of UNRECOVERABLE.
  run(sb, 0.3, 300, t, 250);
  EXPECT_EQ(sb.state(), StallBreaker::State::IDLE);
  EXPECT_EQ(sb.consecutive_pulses(), 0);
}

TEST(StallBreaker, DirectionPreservedNegative) {
  StallBreaker sb("R");
  sb.setConfig(defaultCfg());
  ros::Time t(1000.0);
  run(sb, -0.2, 0, t, 400);
  ASSERT_EQ(sb.state(), StallBreaker::State::PULSING);
  t += ros::Duration(0.02);
  EXPECT_DOUBLE_EQ(sb.update(-0.2, 0, t), -0.8);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
